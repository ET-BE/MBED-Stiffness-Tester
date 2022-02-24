#ifndef MOTOR_H_
#define MOTOR_H_

#include <mbed.h>

/**
 * Basic motor class to combine PWM and direction pin.
 */
class Motor {

public:

    /**
     * Constructor.
     */
    Motor(PinName pin_pwm, PinName pin_dir);

    /**
     * Set motor speed.
     *
     * @param speed     Target speed [-1, 1], output will be clamped
     */
    void set(float speed);

private:
    PwmOut pwm_;
    DigitalOut dir_;
};

#endif
