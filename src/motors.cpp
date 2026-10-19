#include "motors.h"
#include "config.h"

static void writePwm(uint8_t pin, uint8_t ch, uint8_t duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(pin, duty);
#else
    ledcWrite(ch, duty);
#endif
}

void motorsInit() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(PIN_MOTOR_LEFT_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_LEFT_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_RIGHT_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_RIGHT_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
#else
    ledcSetup(LEDC_CH_LEFT_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    
    ledcAttachPin(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1);
    ledcSetup(LEDC_CH_LEFT_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2);
    ledcSetup(LEDC_CH_RIGHT_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1);
    ledcSetup(LEDC_CH_RIGHT_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2);
#endif
    motorsStop();
}

static bool activeBrakeEnabled = true;

void motorsSetActiveBrake(bool enable) {
    activeBrakeEnabled = enable;
}

static void applyMotor(uint8_t in1Pin, uint8_t in1Ch, uint8_t in2Pin, uint8_t in2Ch, int pwm, bool invert) {
    if (invert) pwm = -pwm;
    pwm = constrain(pwm, -255, 255);

    if (pwm > 0) {
        writePwm(in1Pin, in1Ch, (uint8_t)pwm);
        writePwm(in2Pin, in2Ch, 0);
    } else if (pwm < 0) {
        writePwm(in1Pin, in1Ch, 0);
        writePwm(in2Pin, in2Ch, (uint8_t)(-pwm));
    } else {
        // Khi pwm == 0: Kiểm tra cờ activeBrakeEnabled từ Menu OLED
        if (activeBrakeEnabled) {
            // Hãm động năng (Active Dynamic Brake) giúp ghìm tốc độ bánh trong khi vào cua
            writePwm(in1Pin, in1Ch, 255);
            writePwm(in2Pin, in2Ch, 255);
        } else {
            // Thả trôi tự do (Coast)
            writePwm(in1Pin, in1Ch, 0);
            writePwm(in2Pin, in2Ch, 0);
        }
    }
}

void motorsSet(int leftPwm, int rightPwm) {
    applyMotor(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1, PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2, leftPwm, INVERT_LEFT_MOTOR);
    applyMotor(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, rightPwm, INVERT_RIGHT_MOTOR);
}

void motorsStop() {
    // Coast / Float (Low power, no coil buzzing at standstill)
    writePwm(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1, 0);
    writePwm(PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2, 0);
    writePwm(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, 0);
    writePwm(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, 0);
}

void motorsBrake() {
    // Active low-side dynamic brake
    writePwm(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1, 255);
    writePwm(PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2, 255);
    writePwm(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, 255);
    writePwm(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, 255);
}

