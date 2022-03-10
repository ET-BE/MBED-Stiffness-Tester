#include <mbed.h>

#include "main.h"

/**
 * Override built-in console command to enable `printf()`-like commands
 */
FileHandle *mbed::mbed_override_console(int) {
    return &console;
}

//
float potmeter_to_amplitude(float pot) {

    const float dead = 0.2f;

    pot *= 1.0f + dead; // Now range is [0, 1.5]

    if (abs(pot) < dead) {
        return 0.0f;
        // Dead zone to make pausing easier
    }

    float speed;
    if (pot > 0.0f) {
        speed = pot - dead; // range [0, 1]
    } else {
        speed = pot + dead; // range [-1, 0]
    }

    return speed;
}