// #define ADD_MOTION_NOISE
#define CHECK_MAP_BOUNDS

#define TAU 6.28318530718f
#define NUM_PARTICLES 2050
#define NUM_BEACONS 4
#define RSSI_NOISE_STD 2.0f
#define RSSI0 -40.0f       // Reference RSSI at 1m
#define PATH_LOSS_EXPONENT 2.0f
#define MOTION_NOISE_STD 0.01f
#define TIME_STEP 1.0f
#define SIMULATION_STEPS 50

typedef struct {
    float x;
    float y;
} Point;

typedef struct {
    Point min;
    Point max;
} Area;

typedef struct {
    float xx; // Var(x)
    float yy; // Var(y)
    float xy; // Cov(x, y) = Cov(y, x)
} Cov;

void initialize_particle_filter(
        const Area *map,
        float particle_x[], float particle_y[], float particle_w[]
);

void predict_motion(
    const Point *v, const Area *map, float dt,
    float particle_x[], float particle_y[]
);

void update_weights(
    float beacon_x[], float beacon_y[],
    float particle_x[], float particle_y[], float particle_w[],
    float beacon_ref_rssi[], float beacon_path_loss[], float beacon_std[],
    float measurement_RSSI[],
    float temp_weights[]
);

void resample_particles(float particle_x[], float particle_y[], float particle_w[]);

Point estimate_position(float particle_x[], float particle_y[], float particle_w[]);

Cov covariance_matrix(float particle_x[], float particle_y[], float particle_w[], Point position_estimate);

float effective_sample_size(float particle_w[]);

int test_particle_filter(void);