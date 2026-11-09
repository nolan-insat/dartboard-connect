/* 
* TDOA Localization Implementation 
* This source file implements the Time Difference of Arrival (TDOA) localization algorithm using a least-squares approach with 6 sensors.
* Made by: Nolan Reynier Nomer
* Date: 2026-11-08
*/

#include "tdoa.h"

#include <math.h>
#include <stddef.h>

static float distance(
    float x1, float y1,
    float x2, float y2
) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

static void compute_residuals(Point estimate, const TDOA_Config* config, const float delta_t[NB_SENSORS], float residuals[NB_SENSORS -1]) {

    float d1 = distance(
        estimate.x, estimate.y, 
        config->sensors[0].x, config->sensors[0].y);
    
    for (int i = 1; i < NB_SENSORS; ++i) {
        float dist = distance(
            estimate.x, estimate.y,
            config->sensors[i].x, config->sensors[i].y
        );
        residuals[i - 1] = dist - d1 - config->wave_speed * delta_t[i];
    }
}

static float compute_cost(const float residuals[NB_SENSORS - 1]) {
    float cost = 0.0f;
    for (int i = 0; i < NB_SENSORS - 1; ++i) {
        cost += residuals[i] * residuals[i];
    }
    return cost;
}

// -----------------------------------------------------------------------------
// Function to perform TDOA localization using a least-squares approach
// -----------------------------------------------------------------------------



TDOA_Result tdoa_localization(
    const uint32_t timestamps[NB_SENSORS],
    TDOA_Config config,
    Point estimate
) {
    TDOA_Result result = {0};

    if (timestamps == NULL ||
        config.wave_speed <= 0.0f ||
        config.max_iterations <= 0 ||
        config.convergence_threshold <= 0.0f) {
        return result;
    }

    // Time differences relative to sensor 1, in microseconds
    float delta_t[NB_SENSORS] = {0.0f};

    for (int i = 1; i < NB_SENSORS; ++i) {
        int64_t difference =
            (int64_t)timestamps[i] - (int64_t)timestamps[0];

        delta_t[i] = (float)difference;
    }

    Point p = estimate;

    for (int iter = 0; iter < config.max_iterations; ++iter) {

        float residuals[NB_SENSORS - 1] = {0.0f};
        compute_residuals(p, &config, delta_t, residuals);

        float cost = compute_cost(residuals);

        float Hxx = 0.0f, Hxy = 0.0f, Hyy = 0.0f;
        float gx = 0.0f, gy = 0.0f;

        float d1 = distance(
            p.x, p.y,
            config.sensors[0].x,
            config.sensors[0].y
        );

        if (d1 < SINGULARITY_TOLERANCE) {
            break;
        }

        bool singular = false;

        // Build the Jacobian and normal equations
        for (int i = 1; i < NB_SENSORS; ++i) {

            float xi = config.sensors[i].x;
            float yi = config.sensors[i].y;

            float di = distance(p.x, p.y, xi, yi);

            if (di < SINGULARITY_TOLERANCE) {
                singular = true;
                break;
            }

            float jx =
                (p.x - xi) / di
                - (p.x - config.sensors[0].x) / d1;

            float jy =
                (p.y - yi) / di
                - (p.y - config.sensors[0].y) / d1;

            float ri = residuals[i - 1];

            Hxx += jx * jx;
            Hxy += jx * jy;
            Hyy += jy * jy;

            gx += ri * jx;
            gy += ri * jy;
        }

        if (singular) {
            break;
        }

        // Check gradient convergence
        float gradient_norm = sqrtf(gx * gx + gy * gy);

        if (gradient_norm < GRADIENT_TOLERANCE) {
            result.converged = true;
            result.iterations = iter;
            break;
        }

        // Solve the 2x2 normal equations
        float det = Hxx * Hyy - Hxy * Hxy;

        if (fabsf(det) < SINGULARITY_TOLERANCE) {
            break;
        }

        float dx = (Hyy * gx - Hxy * gy) / det;
        float dy = (Hxx * gy - Hxy * gx) / det;

        float step_norm = sqrtf(dx * dx + dy * dy);

        // Check whether the proposed step is already sufficiently small
        if (step_norm < config.convergence_threshold) {
            result.converged = true;
            result.iterations = iter;
            break;
        }

        // Backtracking line search
        float alpha = 1.0f;

        Point old_estimate = p;
        Point new_estimate = p;

        bool accepted = false;

        for (int ls = 0; ls < MAX_LINE_SEARCH_ITERATIONS; ++ls) {

            new_estimate.x = old_estimate.x - alpha * dx;
            new_estimate.y = old_estimate.y - alpha * dy;

            float new_residuals[NB_SENSORS - 1] = {0.0f};

            compute_residuals(
                new_estimate,
                &config,
                delta_t,
                new_residuals
            );

            float new_cost = compute_cost(new_residuals);

            float displacement = alpha * step_norm;

            // Accept a genuine cost decrease.
            // Also allow numerical equality when the step is tiny.
            float cost_tolerance = 1e-6f * (1.0f + cost);

            if (new_cost < cost ||
                (displacement < config.convergence_threshold &&
                 new_cost <= cost + cost_tolerance)) {

                accepted = true;
                break;
            }

            alpha *= 0.5f;
        }

        if (!accepted) {
            break;
        }

        float actual_dx = new_estimate.x - old_estimate.x;
        float actual_dy = new_estimate.y - old_estimate.y;

        float displacement = sqrtf(
            actual_dx * actual_dx +
            actual_dy * actual_dy
        );

        p = new_estimate;

        result.iterations = iter + 1;

        if (displacement < config.convergence_threshold) {
            result.converged = true;
            break;
        }
    }

    // Calculate final residual error
    float final_residuals[NB_SENSORS - 1] = {0.0f};

    compute_residuals(p, &config, delta_t, final_residuals);

    float final_cost = compute_cost(final_residuals);

    result.position = p;
    result.error = sqrtf(final_cost / (NB_SENSORS - 1));

    return result;
}
