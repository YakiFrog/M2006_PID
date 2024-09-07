#include <Arduino.h>
#include "pid.h"
#include "m2006_twai.h"
#include "esp_intr_alloc.h"

// #define DEBUG_MODE
#define OUTPUT_CSV
#define MAX_YAW_RPM 250
#define MAX_AMPERE 3000

float target_degree[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float target_speed[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float target_torque[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int16_t calc_current_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t calc_torque_data[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // デバッグ用
int8_t send_current_data1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int8_t send_current_data2[8] = {0, 0, 0, 0, 0, 0, 0, 0};

float dt = 0;
float prev_time = 0;
float Kt = 0.18 * 1000; // トルク定数[Nm/mA]

float Kp_speed = 100; // BEST: 60
float Ki_speed = 400;
float Kd_speed = 0.04;

float Kp_torque = Kp_speed * 2.75;
float Ki_torque = Ki_speed * 2.75;
float Kd_torque = Kd_speed * 2.75;

float Kp_vrft = 0.49;
float Ki_vrft = 2.48;
float Kd_vrft = 0.03;

// 2次遅れ系のパラメタ
const float K = 1.0;
const float T1_ = 0.2;
const float T2_ = 0.2;

float input_speed[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float output_speed[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float x1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float x2[8] = {0, 0, 0, 0, 0, 0, 0, 0};

float dx1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float dx2[8] = {0, 0, 0, 0, 0, 0, 0, 0};

bool output_csv = false;
float count_time = 0.00;

TaskHandle_t thp[3]; // 3つのタスクを作成
PIDController pid_speed_P(Kp_speed, 0, Kd_speed);
PIDController pid_speed_I(0, Ki_speed, Kd_speed);
PIDController pid_torque(Kp_torque, Ki_torque, Kd_torque);
PIDController pid_vrft(0, 0, 0);
CANWrapper can_wrapper;

void task1(void *args);
void task2(void *args);
void make_current_data(int16_t current_data_in[8], int8_t current_data_out1[8], int8_t current_data_out2[8]);

void setup() {
    Serial.begin(115200);
    // CAN
    while (!can_wrapper.begin()) {
        Serial.println("Failed to start CAN");
    }
    Serial.println("CAN started");
    // Task
    xTaskCreatePinnedToCore(task1, "task1", 4096, NULL, 1, &thp[0], 0); // Serial Monitor
    xTaskCreatePinnedToCore(task2, "task2", 4096, NULL, 1, &thp[1], 0); // Serial Input
}

// Calc PID, Send CAN
void loop() {
    can_wrapper.update();
    /* 使える変数
    1. m_degree[8]
    2. m_rpm[8]
    3. m_torque[8] 
    */

    /* --- コントローラ --- */ 
    dt = (millis() - prev_time) / 1000.0; // [s]
    prev_time = millis();

    // // target_speedを2次遅れ系に通す
    // for (int i = 0; i < 8; i++) {
    //     input_speed[i] = target_speed[i];
    //     dx1[i] = (K * input_speed[i] - x1[i]) / T1_;
    //     dx2[i] = (x1[i] - x2[i]) / T2_;
    //     x1[i] += dx1[i] * dt;
    //     x2[i] += dx2[i] * dt;
    //     output_speed[i] = x2[i];
    // }
    for (int i = 0; i < 8; i++) {
        output_speed[i] = target_speed[i];
    }

    // I-P制御
    for (int i = 0; i < 8; i++) {
        target_torque[i] = (float)pid_speed_I.calculate(output_speed[i], m_rpm[i], dt);
        target_torque[i] -= m_rpm[i] * Kp_speed;
        target_torque[i] = target_torque[i] * (1.0 / Kt); // [A]
    }

    for (int i = 0; i < 8; i++) {
        calc_current_data[i] = (int16_t)pid_torque.calculate(target_torque[i], m_torque[i], dt);
    }
    /* ------------------ */

    /* --- プラント ------ */
    make_current_data(calc_current_data, send_current_data1, send_current_data2);
    can_wrapper.sendMessage(0x200, send_current_data1);
    /* ------------------ */

    delay(10); // 1/5 時定数以下（あげると急な変化に対応できない）
}

void make_current_data(int16_t current_data_in[8], int8_t current_data_out1[8], int8_t current_data_out2[8]) {
    // 0x200用のデータを作成（ID1-4）
    for (int i = 0; i < 4; i++) {
        current_data_out1[0 + (i * 2)] = (current_data_in[i] >> 8) & 0xFF;
        current_data_out1[1 + (i * 2)] = current_data_in[i] & 0xFF;
    }
    // 0x1FF用のデータを作成（ID5-8）
    for (int i = 0; i < 4; i++) {
        current_data_out2[0 + (i * 2)] = (current_data_in[i + 4] >> 8) & 0xFF;
        current_data_out2[1 + (i * 2)] = current_data_in[i + 4] & 0xFF;
    }
}

// Serial Monitor
void task1(void *args) {
    while (1) {
        #ifdef DEBUG_MODE
        Serial.println(">target_speed[0]: " + String(target_speed[0]));
        Serial.println(">target_torque[0]: " + String(target_torque[0]));
        Serial.println(">calc_current_data[0]: " + String(calc_current_data[0]));
        Serial.println(">m_rpm[0]: " + String(m_rpm[0]));
        Serial.println(">m_torque[0]: " + String(m_torque[0]));
        Serial.println(">m_torque[0]: " + String(m_torque[0]));
        Serial.println(">m_degree[0]: " + String(m_degree[0]));
        delay(100);
        #endif
        #ifdef OUTPUT_CSV
        // t, u, y
        if (!output_csv) {
            delay(1);
            continue;
        }
        if (count_time > 5) { // 5秒間のデータを出力
            output_csv = false;
            count_time = 0;
            continue;
        }
        Serial.print(String(count_time) + ",");
        Serial.print(String(output_speed[0]) + ",");
        Serial.print(String(m_rpm[0]));
        Serial.println();
        count_time += 0.010;
        delay(10);
        #endif
    }
}

// Serial Input
void task2(void *args) {
    while (1) {
        // #ifdef DEBUG_MODE
        // 1個目のモータの速度, Kp, Ki, Kdを入力
        if (Serial.available() > 0) {
            // [,]で区切る
            String str = Serial.readStringUntil('\n');
            float input[4];
            for (int i = 0; i < 4; i++) {
                input[i] = str.substring(0, str.indexOf(",")).toFloat();
                str = str.substring(str.indexOf(",") + 1);
            }

            target_speed[0] = input[0];
            Kp_speed = input[1];
            Ki_speed = input[2];
            Kd_speed = input[3];

            Kp_torque = Kp_speed * 2.75;
            Ki_torque = Ki_speed * 2.75;
            Kd_torque = Kd_speed * 2.75;

            pid_speed_P.set_gain(Kp_speed, 0, Kd_speed);
            pid_speed_I.set_gain(0, Ki_speed, Kd_speed);
            // Kp_vrft = input[1];
            // Ki_vrft = input[2];
            // Kd_vrft = input[3];
            // pid_vrft.set_gain(Kp_vrft, Ki_vrft, Kd_vrft);

            output_csv = true;
        }
        // #endif
        delay(100);
    }
}