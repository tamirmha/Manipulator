#ifndef _servo_h
#define _servo_h
#include <Arduino.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>

#define SERVO_MIN_PULSEWIDTH_US 500    // Minimum pulse width for servo motor
#define SERVO_MAX_PULSEWIDTH_US 2500   // Maximum pulse width for servo motor
#define SERVO_MAX_DEGREE 180           // Maximum angle in degrees servo motor can rotate
#define PWM_FREQ 50

#define NUM_MOTORS 7

// Define motor configurations
struct MotorConfig {
  mcpwm_unit_t mcpwm_unit;
  mcpwm_timer_t mcpwm_timer;
  int pwm_pin;
  mcpwm_io_signals_t MCPWM;
  // Add any other necessary configurations
};

// Define motor configurations for each motor
MotorConfig motorConfigs[NUM_MOTORS] = {
    {MCPWM_UNIT_0, MCPWM_TIMER_0, 18, MCPWM0A},
    {MCPWM_UNIT_0, MCPWM_TIMER_1, 5, MCPWM1A},
    {MCPWM_UNIT_0, MCPWM_TIMER_2, 17, MCPWM2A},
    {MCPWM_UNIT_1, MCPWM_TIMER_0, 16, MCPWM0A},
    {MCPWM_UNIT_1, MCPWM_TIMER_1, 4, MCPWM1A},
    {MCPWM_UNIT_1, MCPWM_TIMER_2, 2, MCPWM2A},  //  change to 19
    {MCPWM_UNIT_1, MCPWM_TIMER_2, 15, MCPWM2B}
};


void setup_servo() {
     for (int i = 0; i < NUM_MOTORS; i++) {
        mcpwm_gpio_init(motorConfigs[i].mcpwm_unit, motorConfigs[i].MCPWM, motorConfigs[i].pwm_pin);
        mcpwm_config_t pwm_config;
        pwm_config.frequency = PWM_FREQ;        // Set frequency of PWM signal
        pwm_config.cmpr_a = 0;                  // Initial duty cycle
        pwm_config.cmpr_b = 0;                  // Initial duty cycle
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

        mcpwm_init(motorConfigs[i].mcpwm_unit, motorConfigs[i].mcpwm_timer, &pwm_config); // Initialize PWM module
        mcpwm_start(motorConfigs[i].mcpwm_unit, motorConfigs[i].mcpwm_timer);     
     }
}

void servo_control(int angle, int motorIndex) {
    uint32_t pulse_width_us = map(angle, 0, SERVO_MAX_DEGREE, SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US);
    pulse_width_us = constrain(pulse_width_us, SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US);
    mcpwm_operator_t op = MCPWM_OPR_A;
    if (motorIndex == 6)
        op = MCPWM_OPR_B;
    
    mcpwm_set_duty_in_us(motorConfigs[motorIndex].mcpwm_unit, motorConfigs[motorIndex].mcpwm_timer, op, pulse_width_us);
}

// void servo_loop(bool clockwise)
// {
//     if (clockwise)
//     {
//         for (int i = 0; i <= SERVO_MAX_DEGREE; i++) {
//             for (int j=0; j < NUM_MOTORS; j++)
//                 servo_control(i, j);
//             delay(10); // Adjust speed of rotation
//         }
//     }
//     else
//     {
//         for (int i = SERVO_MAX_DEGREE; i >= 0; i--) {
//             for (int j=0; j < NUM_MOTORS; j++)
//                 servo_control(i, j);
//             delay(10); // Adjust speed of rotation
//         }
//     }
//     delay(10); // Pause at the end of rotation
// }

int remotexy2angle(int val)
{
    return map(val, -100, 100, 0, SERVO_MAX_DEGREE);
}
#endif