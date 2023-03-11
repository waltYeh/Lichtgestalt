#ifndef MOTOR_PWM
#define MOTOR_PWM

void motor_init(void);
void motor_pwm_output(const unsigned short duty[4]);
void motor_cut(void);
#endif
