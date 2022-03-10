#include <mbed.h>

/**
 * Get a nice PWM value [-1, 1] from the potmeter [0, 1]
 */
float potmeter_to_amplitude(float pot);

extern BufferedSerial console;
