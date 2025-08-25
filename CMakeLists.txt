cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Cricchetto_board)

target_sources(app PRIVATE
  src/main.c
  src/observer.c
  src/particle_filter.c
)
