#ifndef __PID__
#define __PID__

/*
MADE BY KOTANI RYOTA
*/

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd), error_sum_(0), prev_error_(0) {}

    float calculate(float setpoint, float current_value, float dt) {
        float error = setpoint - current_value; // 偏差
        error_sum_ += error * dt; // 積分
        float error_diff = (error - prev_error_) / dt; // 微分
        prev_error_ = error;

        // 結果が正であれば正の値を，負であれば負の値を返す
        float output = kp_ * error + ki_ * error_sum_ + kd_ * error_diff;
        return output;
    }

    // Kp, Ki, Kdを変更
    void set_gain(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void reset_error() {
        error_sum_ = 0;
        prev_error_ = 0;
    }

private:
    float kp_;
    float ki_;
    float kd_;
    float error_sum_;
    float prev_error_;
};

#endif // __PID__

// ----------------------------------------------------------------
// Example usage
// ----------------------------------------------------------------

// #include "pid.h"
// #include "Arduino.h"

// PIDController pid(0.5, 0.1, 0.2);

// void setup() {
//     Serial.begin(115200);
// }

// void loop() {
//     float setpoint = 10;
//     float current_value = 0;
//     float dt = 0.1;// [s]
//     pid.set_gain(0.5, 0.1, 0.2); // Kp, Ki, Kd
//     float output = pid.calculate(setpoint, current_value, dt);
//     Serial.println(output);
//     delay(100);
// }
// ----------------------------------------------------------------