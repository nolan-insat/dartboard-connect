
#include "tdoa.h"

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define PI 3.14159265f

int main(void)
{
    // ------------------------------------------
    // Configuration des 6 capteurs
    // ------------------------------------------

    TDOA_Config config = {0};

    float radius = 200.0f;

    for (int i = 0; i < NB_SENSORS; i++) {

        float angle = 2.0f * PI * i / NB_SENSORS;

        config.sensors[i].x =
            radius * cosf(angle);

        config.sensors[i].y =
            radius * sinf(angle);
    }

    config.wave_speed = 2.0f;
    config.max_iterations = MAX_ITERATIONS;
    config.convergence_threshold = 1e-3f;

    // ------------------------------------------
    // Position réelle de l'impact
    // ------------------------------------------

    Point real_position = {
        .x = 100.0f,
        .y = -100.0f
    };

    float t0 = 100000.0f;

    uint32_t timestamps[NB_SENSORS];

    // ------------------------------------------
    // Simulation des 6 temps d'arrivée
    // ------------------------------------------

    for (int i = 0; i < NB_SENSORS; i++) {

        float dx = real_position.x - config.sensors[i].x;
        float dy = real_position.y - config.sensors[i].y;

        float d = sqrtf(dx * dx + dy * dy);

        float arrival_time =
            t0 + d / config.wave_speed;

        timestamps[i] = (uint32_t)roundf(arrival_time);

        printf("Capteur %d : %u us\n",
               i + 1, timestamps[i]);
    }

    // ------------------------------------------
    // Appel de l'algorithme TDOA
    // ------------------------------------------

    Point initial_guess = {0.0f, 0.0f};

    TDOA_Result result = tdoa_localization(
        timestamps,
        config,
        initial_guess
    );

    // ------------------------------------------
    // Affichage des résultats
    // ------------------------------------------

    printf("\n--- RESULTATS ---\n");

    if (result.converged) {

        printf("Position reelle : (%.2f, %.2f) mm\n",
               real_position.x, real_position.y);

        printf("Position estimee : (%.2f, %.2f) mm\n",
               result.position.x, result.position.y);

        printf("Erreur RMS : %.4f mm\n",
               result.error);

        float error = sqrtf(
            powf(result.position.x - real_position.x, 2)
            +
            powf(result.position.y - real_position.y, 2)
        );

        printf("Erreur position : %.4f mm\n", error);

        printf("Iterations : %u\n", result.iterations);
    }
    else {
        printf("Echec de la localisation\n");
    }

    return 0;
}
