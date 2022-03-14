#include <chrono>
#include <cstdio>
#include <mbed.h>

#include "main.h"

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

BufferedSerial console(STDIO_UART_TX, STDIO_UART_RX, 115200);

DigitalOut led(LED_BLUE);

Motor motor(D5, D4);

//PID pid(0.02f, 0.002f, 0.0f, 1.0f / Fs);
PID pid(0.5f, 0.02f, 0.01f, 1.0f / Fs);

QEI encoder(D13, D12, 8384);
AnalogIn pot_meter(A0);

HX711 loadcell(D2, D3); // data, clock

BiquadFilter filter_torque(Fs, 10.0f, BiquadFilter::TYPE_LOW_PASS);

float max_angle = 10.0f; // Degrees
float max_torque = 0.2; // Nm

int main()
{
    t.start(); // Main loop timer
    duration<float> loop_time(1.0f / Fs);

    pid.setFilter(75.0f); // Add filter for derivative

    // The amplifier is 24 bit
    // (int rangesensor * amplifier gain) / (Vcc * sensitivity)
    loadcell.setScale((8388608.0f * 128.0f) / (5.0f * 853.0f));
    loadcell.powerUp();
    loadcell.tare(50); // Perform tare with many measurements (blocking)

    float torque = 0.0f;
    float ref_angle = 0.0f;

    printf("\nSetting up scope...\n");
    ThisThread::sleep_for(1ms);

    HIDScope scope(4);

    printf("Scope connected.\n");

    printf("\nChange settings by typing: [letter][value]\n");
    printf("Use: a - Max angle, t - Torque limit\n");

    while (true) {
        auto next = t.elapsed_time() + loop_time;

        // ---- loop ----

        if (loadcell.isReady()) {
            torque = loadcell.getUnits(1); // Get a single measurement
        }

        float pot_angle = potmeter_to_amplitude(pot_meter.read())
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

        motor.set(pwm);

        // Also update filter when no data is coming in        
        float torque_filtered = filter_torque.sample(torque);

        // Print
        static unsigned int counter = 0;
        if (counter++ > 100) {

            static bool blink = false;
            led.write(blink = !blink);
            //printf("Speed: %.2f - Angle: %.1f - Torque: %.1f\n", speed, angle, torque);
            counter = 0;
        }

        // Send data
        unsigned int i = 0;
        scope.set(i++, ref_angle);
        scope.set(i++, angle);
        scope.set(i++, pwm);
        scope.set(i++, torque_filtered);
        scope.send();

        // Process serial input

        static char c;
        static char input_buffer[32];
        static unsigned int input_i = 0;

        while (console.readable()) {
            console.read(&c, 1);
            
            if (input_i >= 32 || c == 13) {

                printf("\n");
                if (input_i > 2) {

                    input_buffer[input_i] = '\0'; // Insert null terminator

                    char var;
                    float val;
                    sscanf(input_buffer, "%c%f", &var, &val);

                    if (var == 'a') {
                        if (val > 0.01f && val < 30.0f) {
                            max_angle = val;
                        }
                        printf("New max. angle: %.2f\n", max_angle);
                    } else if (var == 't') {
                        if (val > 0.01f && val < 1.0f) {
                            max_torque = val;
                        }
                        printf("New torque limit: %.2f\n", max_torque);
                    } else {
                        printf("Unknown command [%c]\n", var);
                    }
                }

                input_i = 0;
                break;
            } else {
                console.write(&c, 1); // Echo back
            }

            input_buffer[input_i++] = c;
        }

        // Make sure the execution time + pause time == loop time
        while (t.elapsed_time() < next) {}
    }
}
