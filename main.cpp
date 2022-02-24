#include <chrono>
#include <mbed.h>

#include "motor/motor.h"
#include "quadrature-encoder/QEI.h"
#include "HX711/HX711.h"
#include "pid/pid.h"
#include "uscope/scope_serial.h"
#include "uscope/scope_hid.h"
#include "biquad-filter/biquad_filter.h"

using namespace std::chrono;

const float Fs = 500; // Hz

Timer t;

DigitalOut led(LED_BLUE);

Motor motor(D5, D4);

PID pid(0.02f, 0.002f, 0.0f, 1.0f / Fs);

QEI encoder(D13, D12, 8384);
AnalogIn pot_meter(A0);

HX711 loadcell(D2, D3); // data, clock

BiquadFilter filter_torque(Fs, 10.0f, BiquadFilter::TYPE_LOW_PASS);

const float max_angle = 10.0f; // Degrees
const float max_torque = 0.1; // Nm

/**
 * Get a nice PWM value [-1, 1] from the potmeter [0, 1]
 */
float potmeter_to_pwm(float pot) {

    pot = 2.0f * pot_meter.read() - 1.0f; // Now ranges from [-1, 1]

    const float dead = 0.4f;

    pot *= 1.0f + dead; // Now range is [-1.5, 1.5]

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

int main()
{
    t.start(); // Main loop timer
    duration<float> loop_time(1.0f / Fs);

    pid.setFilter(75.0f); // Add filter for derivative

    float startup = 0.0f;

    // The amplifier is 24 bit
    // (int rangesensor * amplifier gain) / (Vcc * sensitivity)
    loadcell.setScale((8388608.0f * 128.0f) / (5.0f * 853.0f));
    loadcell.powerUp();
    loadcell.tare(50); // Perform tare with many measurements (blocking)

    float torque = 0.0f;
    float ref_angle = 0.0f;

    printf("Setting up scope...\n");
    ThisThread::sleep_for(1ms);

    HIDScope scope(3);

    printf("Scope connected.\n");

    while (true) {
        auto next = t.elapsed_time() + loop_time;

        // ---- loop ----

        if (loadcell.isReady()) {
            torque = loadcell.getUnits(1); // Get a single measurement
        }

        float pot_angle = potmeter_to_pwm(pot_meter.read())
            * max_angle; // Get desired deflection

        // float ref_angle_new = pot_angle;

        const float freq = 1.0f / 5.0f; // 1 / period
        float time = (duration<float>(t.elapsed_time())).count();
        float ref_angle_new = pot_angle * sinf(2.0f * M_PI * freq * time);

        if (abs(torque) < max_torque ||
                abs(ref_angle_new) < abs(ref_angle)) {

            // Accept only safe values
            ref_angle = ref_angle_new;
        }

        float angle = encoder.getRevolutions() * 360.0f;

        float pwm = pid.control(ref_angle - angle);

        if (abs(pwm) < 0.01f) {
            pwm = 0.0f; // Prevent stalling with annoying sound
        }

        motor.set(pwm);

        // Also update filter when no data is coming in        
        float torque_filtered = filter_torque.sample(torque);

        // Print
        static unsigned int counter = 0;
        if (counter++ > 100) {

            led.write(led.read());
            //printf("Speed: %.2f - Angle: %.1f - Torque: %.1f\n", speed, angle, torque);
            counter = 0;
        }

        // Send data
        unsigned int i = 0;
        scope.set(i++, angle);
        scope.set(i++, pwm);
        scope.set(i++, torque_filtered);
        scope.send();

        // Make sure the execution time + pause time == loop time
        while (t.elapsed_time() < next) {}
    }
}
