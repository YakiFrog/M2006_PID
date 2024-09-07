#ifndef M2006_TWAI_H
#define M2006_TWAI_H

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/twai.h>

#define RX_GPIO_PIN GPIO_NUM_4
#define TX_GPIO_PIN GPIO_NUM_5

// RXD GPIO 4
// TXD GPIO 5

#define DEFAULT_INTERVAL 0

// 受信したデータを格納する変数
// 場所がわかりにくくてごめんね．ポインタ使えばもっと綺麗に書けるかもね
int16_t m_degree[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t m_rpm[8]    = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t m_torque[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // トルクといいつつ，実際は電流(mA)

class CANWrapper {
public:
  using LogCallback = void (*)(const String& message);

  CANWrapper(LogCallback logCallback = nullptr) : driver_installed(false), logCallback(logCallback) {}

  bool begin() {
    if (!initializeDriver()) {
      log("Failed to initialize CAN driver");
      return false;
    }

    if (!startDriver()) {
      log("Failed to start CAN driver");
      return false;
    }

    if (!configureAlerts()) {
      log("Failed to configure CAN alerts");
      return false;
    }

    driver_installed = true;
    return true;
  }
  bool waitForConnection() {
    while (!driver_installed) {
      delay(DEFAULT_INTERVAL);
    }
    return true;
  }

  void update() {
    if (!driver_installed) {
      delay(DEFAULT_INTERVAL);
      return;
    }

    processAlerts(); 
    printStatusInfo();
  }

  void sendMessage(uint32_t identifier, int8_t data[], uint8_t data_length = TWAI_FRAME_MAX_DLC) {
    if (driver_installed) {
      twai_message_t tx_message;
      tx_message.identifier = identifier;
      tx_message.data_length_code = data_length;
      memcpy(tx_message.data, data, data_length);
      tx_message.flags = TWAI_MSG_FLAG_NONE;

      twai_transmit(&tx_message, pdMS_TO_TICKS(DEFAULT_INTERVAL));
    }
  }

  void receiveMessage(int16_t degree[8], int16_t rpm[8], int16_t torque[8]) {
    memcpy(degree, m_degree, sizeof(m_degree));
    memcpy(rpm, m_rpm, sizeof(m_rpm));
    memcpy(torque, m_torque, sizeof(m_torque));
  }

private:
  bool driver_installed;
  twai_status_info_t status_info;
  LogCallback logCallback;

  bool initializeDriver() {
    delay(5000);
    log("CAN Receiver");
    
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_PIN, RX_GPIO_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    return twai_driver_install(&general_config, &timing_config, &filter_config) == ESP_OK;
  }

  bool startDriver() {
    return twai_start() == ESP_OK;
  }

  bool configureAlerts() {
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
    return twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK;
  }

  void processAlerts() {
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(DEFAULT_INTERVAL));

    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
      processReceivedMessages();
    } else {
      logNoMessageReceived();
    }

    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
      log("Alert: TWAI controller entered error passive state");
    else if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      log("Alert: Error occurred on the bus (bit, stuff, CRC, form, ACK)");
      log(String("Bus error count: ") + status_info.bus_error_count);
    } else if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      log("Alert: RX queue is full, received frames were lost");
      printRXQueueInfo();
    }

    log(String("TX error count: ") + status_info.tx_error_counter);
    log(String("RX error count: ") + status_info.rx_error_counter);
  }

  void processReceivedMessages() {
    twai_message_t rx_message;
    while (twai_receive(&rx_message, 0) == ESP_OK)
      handleReceivedMessage(rx_message);
  }


  void handleReceivedMessage(twai_message_t& message) {
    String logMessage;
    logMessage += String(message.identifier, HEX);
    logMessage += ": ";
    for (int i = 0; i < message.data_length_code; i++) {
      logMessage += String(message.data[i], HEX);
      logMessage += " ";
    }
    log(logMessage);

    int index = message.identifier - 0x201;
    m_degree[index] = (((message.data[0] << 8) | message.data[1]) * 360) / 8192;
    int16_t rpmValue = (message.data[2] << 8 | message.data[3]);
    if (rpmValue > 32767) {
      m_rpm[index] = rpmValue - 65536;
    } else {
      m_rpm[index] = rpmValue;
    }
    m_rpm[index] /= 36; 
    m_torque[index] = ((message.data[4] << 8) | message.data[5]);
    m_torque[index] /= (float)(0.18 * 1000);

  }

  void logNoMessageReceived() {
    static uint32_t last_alert_check_time = 0;
    uint32_t current_time = millis();
    log("No message received: " + String(current_time - last_alert_check_time) + "ms");
    last_alert_check_time = current_time;
  }

  void printRXQueueInfo() {
    log(String("RX buffering: ") + status_info.msgs_to_rx);
    log(String("RX miss: ") + status_info.rx_missed_count);
    log(String("RX overrun: ") + status_info.rx_overrun_count);
  }

  void printStatusInfo() {
    twai_get_status_info(&status_info);
    log(String("TX error count: ") + status_info.tx_error_counter);
    log(String("RX error count: ") + status_info.rx_error_counter);
  }

  void log(const String& message) {
    if (logCallback) {
      logCallback(message);
    } else {
      // Serial.println(message);
    }
  }
};

#endif // M2006_TWAI_H
