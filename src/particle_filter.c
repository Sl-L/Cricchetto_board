#include "particle_filter.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include "arm_math.h"
#include <stdio.h>
#include <stdlib.h>

static float temp_weights[NUM_PARTICLES];

float rand_uniform(float min, float max) {
    return min + (max - min) * (rand() / (float)RAND_MAX);
}

float rand_normal(float mean, float std) {
    float u1, u2;
    do {
        u1 = rand_uniform(0.0f, 1.0f);
        u2 = rand_uniform(0.0f, 1.0f);
    } while (u1 <= 1e-12f);  // Avoid log(0)

    // Box-Muller transform
    float z = sqrtf(-2.0f * logf(u1)) * cosf(TAU * u2);
    return mean + std * z;
}

void initialize_particle_filter(
        const Area *map,
        float particle_x[], float particle_y[], float particle_w[]
    ) {

    arm_fill_f32(1.0f / NUM_PARTICLES, particle_w, NUM_PARTICLES);

    for (int i = 0; i < NUM_PARTICLES; i++) {
        particle_x[i] = rand_uniform(map->min.x, map->max.x);
        particle_y[i] = rand_uniform(map->min.y, map->max.y);
    }
}

// Move particles based on known velocity
void predict_motion(
    const Point *v, const Area *map, float dt,
    float particle_x[], float particle_y[]
) {

    arm_offset_f32(particle_x, v->x * dt, particle_x, NUM_PARTICLES);
    arm_offset_f32(particle_y, v->y * dt, particle_y, NUM_PARTICLES);
    
    #if defined ADD_MOTION_NOISE || defined CHECK_MAP_BOUNDS
        for (int i = 0; i < NUM_PARTICLES; i++) {
            // Add independent noise to each particle's motion
            #ifdef ADD_MOTION_NOISE
                particle_x[i] += rand_normal(0.0f, MOTION_NOISE_STD);
                particle_y[i] += rand_normal(0.0f, MOTION_NOISE_STD);
            #endif

            // Clamp to map boundaries
            #ifdef CHECK_MAP_BOUNDS
                particle_x[i] = fmaxf(map->min.x, fminf(map->max.x, particle_x[i]));
                particle_y[i] = fmaxf(map->min.y, fminf(map->max.y, particle_y[i]));
            #endif
        }
    #endif
}


void update_weights(
    float beacon_x[], float beacon_y[],
    float particle_x[], float particle_y[], float particle_w[],
    float beacon_ref_rssi[], float beacon_path_loss[], float beacon_std[],
    float measurement_RSSI[],
    float temp_weights[]
) {
    arm_fill_f32(0.0f, temp_weights, NUM_PARTICLES);

    static float d_squared[NUM_PARTICLES];
    static float expected_rssi[NUM_PARTICLES];
    static float weight_update[NUM_PARTICLES];
    const float rssi_scale = - 2.1714724095f * PATH_LOSS_EXPONENT;

    static float dx, dy;
    static float weight_scale;
    for (int i = 0; i < NUM_BEACONS; i++) {
        weight_scale = - 0.5f / (beacon_std[i] * beacon_std[i]);

        // Calculate squared distance
        for (int j = 0; j < NUM_PARTICLES; j++) {
            dx = particle_x[j] - beacon_x[i];
            dy = particle_y[j] - beacon_y[i];

            d_squared[j] = dx*dx + dy*dy;
            if (d_squared[j] < 0.01f) d_squared[j] = 0.01f;
        }
        /* 
        Calculate expected RSSI using squared distances 
        
        log10(sqrt(x**2 + y**2)) = 0.5*log10(x**2 + y**2) = 0.5*(log(x**2 + y**2)/log(10)) = 0.2171472409f * log(x**2 + y**2)
        */
        arm_vlog_f32(d_squared, d_squared, NUM_PARTICLES);
        arm_scale_f32(d_squared, rssi_scale, expected_rssi, NUM_PARTICLES);
        arm_offset_f32(expected_rssi, beacon_ref_rssi[i], expected_rssi, NUM_PARTICLES);
        
        // Calculate Gaussian likelihood
        arm_offset_f32(expected_rssi, -measurement_RSSI[i], weight_update, NUM_PARTICLES);
        arm_mult_f32(weight_update, weight_update, weight_update, NUM_PARTICLES);
        arm_scale_f32(weight_update, weight_scale, weight_update, NUM_PARTICLES);
        
        // Accumulate weights
        arm_add_f32(temp_weights, weight_update, temp_weights, NUM_PARTICLES);
    }

    // Normalize weights
    float max_weight;
    int max_idx;
    arm_max_f32(temp_weights, NUM_PARTICLES, &max_weight, &max_idx);
    arm_offset_f32(temp_weights, -max_weight, temp_weights, NUM_PARTICLES);
    arm_vexp_f32(temp_weights, temp_weights, NUM_PARTICLES);
    
    float sum = 0;

    for (int i = 0; i < NUM_PARTICLES; i++) {
        sum += temp_weights[i];
    }
    
    if (sum > 1e-12f) {
        arm_scale_f32(temp_weights, 1.0f/sum, particle_w, NUM_PARTICLES);
    } else {
        arm_fill_f32(1.0f/NUM_PARTICLES, particle_w, NUM_PARTICLES);
    }
}

int binary_search(float *cumulative_weights, int size, float threshold) {
    int low = 0;
    int high = size - 1;

    while (low < high) {
        int mid = (low + high) / 2;
        if (threshold > cumulative_weights[mid]) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    return low;
}

void resample_particles(float particle_x[], float particle_y[], float particle_w[]) {
    static float new_x[NUM_PARTICLES], new_y[NUM_PARTICLES];
    float cumulative_weights[NUM_PARTICLES];
    
    // Build cumulative weights
    cumulative_weights[0] = particle_w[0];
    for (int i = 1; i < NUM_PARTICLES; i++) {
        cumulative_weights[i] = cumulative_weights[i-1] + particle_w[i];
    }
    
    // Systematic resampling
    float step = 1.0f/NUM_PARTICLES;
    float u = rand_uniform(0.0f, step);
    
    for (int i = 0; i < NUM_PARTICLES; i++) {
        float threshold = u + i*step;
        int idx = binary_search(cumulative_weights, NUM_PARTICLES, threshold);
        new_x[i] = particle_x[idx];
        new_y[i] = particle_y[idx];
    }
    
    // Copy new particles
    arm_copy_f32(new_x, particle_x, NUM_PARTICLES);
    arm_copy_f32(new_y, particle_y, NUM_PARTICLES);
    arm_fill_f32(1.0f/NUM_PARTICLES, particle_w, NUM_PARTICLES);  // Reset weights
}

Point estimate_position(float particle_x[], float particle_y[], float particle_w[]) {
    Point estimate = {0.0f, 0.0f};
    
    estimate.x = arm_weighted_average_f32(particle_x, particle_w, NUM_PARTICLES);
    estimate.y = arm_weighted_average_f32(particle_y, particle_w, NUM_PARTICLES);
    
    return estimate;
}

float effective_sample_size(float particle_w[]) {
    float ees;
    arm_power_f32(particle_w, NUM_PARTICLES, &ees);
    return 1.0f / ees;
}

// Only used in sim
float distance(Point *p1, Point *p2) {
    float dx = p1->x - p2->x;
    float dy = p1->y - p2->y;
    return sqrtf(dx * dx + dy * dy);
}

// Simulate RSSI measurement (with noise)
void get_rssi_measurement(Point *true_pos, Point beacons[], float observed_rssi[]) {
    for (int j = 0; j < NUM_BEACONS; j++) {
        float d = distance(true_pos, &beacons[j]);
        if (d < 0.1f) d = 0.1f;  // Avoid log of very small numbers
        float rssi = RSSI0 - 10.0f * PATH_LOSS_EXPONENT * log10f(d);
        observed_rssi[j] = rssi + rand_normal(0.0f, RSSI_NOISE_STD);
    }
}

int test_particle_filter(void) {
    srand(93420817);

    static float particle_x[NUM_PARTICLES];
    static float particle_y[NUM_PARTICLES];
    static float particle_w[NUM_PARTICLES];

    static float beacon_x[NUM_BEACONS];
    static float beacon_y[NUM_BEACONS];
    static float beacon_std[NUM_BEACONS];
    static float beacon_path_loss[NUM_BEACONS];
    static float beacon_ref_rssi[NUM_BEACONS];

    for (int i = 0; i < NUM_BEACONS; i++) {
        beacon_ref_rssi[i] = RSSI0;
        beacon_path_loss[i] = PATH_LOSS_EXPONENT;
        beacon_std[i] = RSSI_NOISE_STD;
    }

    // Beacon positions
    Point beacons[NUM_BEACONS] = {
        {0.0f, 0.0f}, {10.0f, 0.0f}, {0.0f, 10.0f}, {10.0f, 10.0f}
    };

    for (int i = 0; i < NUM_BEACONS; i++) {
        beacon_x[i] = beacons[i].x;
        beacon_y[i] = beacons[i].y;
    }
    

    // Motion parameters
    const Point v = {0.2f, 0.1f};
    const Area map = {{0.0f, 0.0f}, {10.0f, 10.0f}};

    printk("Initializing particles\n");
    // Initialize particles
    initialize_particle_filter(&map, particle_x, particle_y, particle_w);
    
    printk("Particles ready, starting simulation\n");

    // Simulation variables
    Point true_pos = {0.0f, 0.0f};
    Point estimated_trajectory[SIMULATION_STEPS];

    uint32_t start = k_uptime_get_32();

    uint32_t cumsum_predict = 0;
    uint32_t cumsum_update = 0;
    uint32_t cumsum_resample = 0;
    uint32_t cumsum_estimate = 0;

    uint32_t dt_predict;
    uint32_t dt_update;
    uint32_t dt_resample;
    uint32_t dt_estimate;

    int rs = 0;

    for (int t = 0; t < SIMULATION_STEPS; t++) {
        // Update true position
        true_pos.x += v.x * TIME_STEP;
        true_pos.y += v.y * TIME_STEP;
        
        // Ensure true position stays within bounds
        if (true_pos.x < map.min.x) true_pos.x = map.min.x;
        else if (true_pos.x > map.max.x) true_pos.x = map.max.x;

        if (true_pos.y < map.min.y) true_pos.y = map.min.y;
        else if (true_pos.y > map.max.y) true_pos.y = map.max.y;


        // Simulate RSSI measurements
        float observed_rssi[NUM_BEACONS];
        get_rssi_measurement(&true_pos, beacons, observed_rssi);

        // Particle filter steps
        dt_predict = k_uptime_get_32();
        predict_motion(&v, &map, TIME_STEP, particle_x, particle_y);
        cumsum_predict += k_uptime_get_32() - dt_predict;

        dt_update = k_uptime_get_32();
        update_weights(beacon_x, beacon_y, particle_x, particle_y, particle_w, beacon_ref_rssi, beacon_path_loss, beacon_std, observed_rssi, temp_weights);
        cumsum_update += k_uptime_get_32() - dt_update;

        if (effective_sample_size(particle_w) < (NUM_PARTICLES / 2)) {
            rs += 1;
            dt_resample = k_uptime_get_32();
            resample_particles(particle_x, particle_y, particle_w);
            cumsum_resample += k_uptime_get_32() - dt_resample;
        }
        

        dt_estimate = k_uptime_get_32();
        estimated_trajectory[t] = estimate_position(particle_x, particle_y, particle_w);
        cumsum_estimate += k_uptime_get_32() - dt_estimate;

        //Print results
        printk("Step %2d: True (%.2f, %.2f), Estimated (%.2f, %.2f)\n",
               t, (double)true_pos.x, (double)true_pos.y,
               (double)estimated_trajectory[t].x, (double)estimated_trajectory[t].y);
    }

    uint32_t end = k_uptime_get_32();
    printk("Elapsed time: %d ms\n", end - start);
    printk("Movement prediction time: %d ms\n", cumsum_predict);
    printk("Weight update time: %d ms\n", cumsum_update);
    printk("Particle resample time: %d ms (resamples: %d)\n", cumsum_resample, rs);
    printk("Position estimation time: %d ms\n", cumsum_estimate);

    printk("\nAverages:\n");
    printk("  Total: %d ms\n", (end - start) / SIMULATION_STEPS);
    printk("  Movement prediction: %d ms\n", cumsum_predict / SIMULATION_STEPS);
    printk("  Weight update: %d ms\n", cumsum_update / SIMULATION_STEPS);
    printk("  Particle resample: %d ms\n", cumsum_resample / SIMULATION_STEPS);
    printk("  Position estimation: %d ms\n", cumsum_estimate / SIMULATION_STEPS);

    return 0;
}