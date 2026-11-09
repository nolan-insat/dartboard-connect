/* 
* TDOA Localization 
* This header file defines the structures and function prototype for Time Difference of Arrival (TDOA) localization 
* using a least-squares approach with 6 sensors.
* Made by: Nolan Reynier Nomer
* Date: 2026-11-08
*/
#ifndef TDOA_H
#define TDOA_H

#include <stdint.h>
#include <stdbool.h>

#define NB_SENSORS 6
#define SINGULARITY_TOLERANCE 1e-8f
#define GRADIENT_TOLERANCE 1e-4f
#define MAX_ITERATIONS 100
#define MAX_LINE_SEARCH_ITERATIONS 20

// Structure to represent a point in 2D space in mm.
typedef struct {
    float x;
    float y;
} Point;

// Structure to represent a sensor with its position in 2D space in mm.
typedef struct {
    float x;
    float y;
} Sensor;

typedef struct {
    Sensor sensors[NB_SENSORS]; // Array of 6 sensors
    float wave_speed; // Speed of the wave in mm/us
    float convergence_threshold; // Threshold for convergence in mm
    int max_iterations; // Maximum number of iterations for the algorithm
} TDOA_Config;

// Structure to hold the result of the TDOA localization
typedef struct {
    Point position;
    float error;
    int iterations;
    bool converged;
} TDOA_Result;

// Function prototype for TDOA localization
TDOA_Result tdoa_localization(
    const uint32_t timestamps[NB_SENSORS], 
    TDOA_Config config,
    Point estimate
);

#endif // TDOA_H