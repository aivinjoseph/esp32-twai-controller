#include <Arduino.h>
#include "driver/twai.h"

#define TX_GPIO_NUM 4
#define RX_GPIO_NUM 5

twai_message_t rx_msg;
twai_message_t last_cmd;
bool has_cmd = false;
bool repeat_enabled = true;
bool power_on = false;

enum Step {
  STEP_SELECT_MODE = 0,
  STEP_SET_MODE,
  STEP_INPUT_PHASE,
  STEP_SET_PHASE,
  STEP_POWER_CTRL,
  STEP_POWER_ON
};

// TWAI setup
void setup_twai() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_GPIO_NUM, (gpio_num_t)RX_GPIO_NUM, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI install failed");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed");
    return;
  }
  Serial.println("TWAI started");
}

// Background task to repeat last command every 500ms
void repeat_command_task(void* param) {
  while (true) {
    if (has_cmd && repeat_enabled) {
      twai_transmit(&last_cmd, pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Send command and store as last
void send_command(uint32_t can_id, const uint8_t *data) {
  last_cmd.identifier = can_id;
  last_cmd.data_length_code = 8;
  last_cmd.flags = TWAI_MSG_FLAG_EXTD;
  memcpy(last_cmd.data, data, 8);
  has_cmd = true;
}                                                                                                                                                                                                                   

// Convert float to hex bytes with scaling
void floatToHexBytes(float value, float unit_scale, uint8_t &high, uint8_t &low) {
  uint16_t scaled = static_cast<uint16_t>(value * unit_scale);
  high = (scaled >> 8) & 0xFF;
  low = scaled & 0xFF;
}

// Wait for CAN response
bool check_response(const uint8_t *expected_prefix, int timeout_ms = 500) {
  repeat_enabled = false;
  bool match = false;

  if (twai_receive(&rx_msg, pdMS_TO_TICKS(timeout_ms)) == ESP_OK) {
    Serial.print("Received: ");
    for (int i = 0; i < rx_msg.data_length_code; ++i) {
      Serial.printf("%02X ", rx_msg.data[i]);
    }
    Serial.println();

    if ((rx_msg.identifier & 0x1FFFFFFF) == 0x061F8008 &&
        memcmp(rx_msg.data, expected_prefix, 4) == 0) {
      match = true;
    }
  }

  repeat_enabled = true;
  return match;
}

// Send command with retry attempts
bool send_command_with_retry(uint32_t can_id, const uint8_t *data, const uint8_t *expected_response, int max_retries = 5) {
  for (int attempt = 0; attempt < max_retries; ++attempt) {
    send_command(can_id, data);
    if (check_response(expected_response)) {
      return true;
    }
    Serial.printf("Retry %d/%d failed. Retrying...\n", attempt + 1, max_retries);
    delay(300);
  }
  return false;
}

// Mode command and response data
const uint8_t* get_mode_cmd(int index) {
  static const uint8_t mode_cmds[3][8] = {
    {0x03, 0x00, 0x00, 0x2F, 0x00, 0x01, 0x00, 0x00},
    {0x03, 0x00, 0x00, 0x2F, 0x00, 0x01, 0x00, 0x01},
    {0x03, 0x00, 0x00, 0x2F, 0x00, 0x01, 0x00, 0x02}
  };
  return mode_cmds[index];
}

const uint8_t* get_mode_response(int index) {
  static const uint8_t mode_response[3][8] = {
    {0x42, 0xF0, 0x00, 0x2F, 0x00, 0x01, 0x00, 0x00},
    {0x42, 0xF0, 0x00, 0x2F, 0x00, 0x01, 0x00, 0x01},
    {0x42, 0xF0, 0x00, 0x2F, 0x00, 0x01, 0x00, 0x02}
  };
  return mode_response[index];
}


// Static command data
static const uint8_t phase_cmd_resp[4] = {0x42, 0xF0, 0x00, 0x84};
static const uint8_t power_on_cmd[8]  = {0x03, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00};
static const uint8_t power_off_cmd[8] = {0x03, 0x00, 0x00, 0x30, 0x00, 0x01, 0x00, 0x00};
static const uint8_t power_on_resp[8] = {0x42, 0xF0, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00};
static const uint8_t power_off_resp[8] = {0x42, 0xF0, 0x00, 0x30, 0x00, 0x01, 0x00, 0x00};

// Step state
Step step = STEP_SELECT_MODE;
int selected_mode = 0;    //fixed behavior for mode selection
uint8_t phase_cmd[8];

void handleModeSelection() {

  repeat_enabled = false;
  Serial.println("Select mode: 1-OnGrid (default), 2-OffGrid, 3-Rectifier:");
  while (!Serial.available());
  char ch = Serial.read();
  if (ch == '2') selected_mode = 1;
  else if (ch == '3') selected_mode = 2;
  else if (ch == '1') selected_mode = 0;
  else {
    Serial.println("Invalid selection.");
    return;
  }
    Serial.printf("Selected mode: %d\n", selected_mode + 1);
    Serial.println("Setting mode...");
    repeat_enabled = true;
    step = STEP_SET_MODE;
  
}

void handleSetMode() {
  if (send_command_with_retry(0x06180F81, get_mode_cmd(selected_mode), get_mode_response(selected_mode))) {
    Serial.println("Mode set successfully.");
    step = STEP_INPUT_PHASE;
  } else {
    Serial.println("Failed to set mode after retries.");
  }
}

void handlePhaseVoltage() {
  while (Serial.available() > 0) Serial.read();
  Serial.println("Enter phase voltage (e.g., 230.0 for 230V):");
  while (!Serial.available());
  float voltage_input = Serial.parseFloat();

  if (voltage_input > 420.0) {
    Serial.println("Invalid phase Voltage. Try again.");
    return;
  }

  uint8_t volt_high, volt_low, freq_high, freq_low;
  floatToHexBytes(voltage_input, 10.0, volt_high, volt_low);      // 0.1V scale
  floatToHexBytes(60.0, 1000.0, freq_high, freq_low);              // 60 Hz to mHz

  phase_cmd[0] = 0x03;
  phase_cmd[1] = 0x00;
  phase_cmd[2] = 0x00;
  phase_cmd[3] = 0x84;
  phase_cmd[4] = volt_high;
  phase_cmd[5] = volt_low;
  phase_cmd[6] = freq_high;
  phase_cmd[7] = freq_low;

  step = STEP_SET_PHASE;
}

void handleSetPhase() {
  if (send_command_with_retry(0x06180F81, phase_cmd, phase_cmd_resp)) {
    Serial.println("Phase voltage and 60Hz frequency set successfully.");
    step = STEP_POWER_CTRL;
  } else {
    Serial.println("Failed to set phase voltage and frequency after retries.");
  }
}

void handlePowerControl() {
  while (Serial.available() > 0) Serial.read();
  Serial.println("Type 'on' to power on:");
  while (!Serial.available());
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "on") {
    power_on = true;
    step = STEP_POWER_ON;
  } else {
    Serial.println("Invalid input. Type 'on' or wait at ready.");
  }
}

void handlePowerOnOff() {
  if (send_command_with_retry(0x06180F81, power_on_cmd, power_on_resp)) {
    Serial.println("Power ON successful.");
    while (Serial.available() > 0) Serial.read();
    Serial.println("Type 'off' to power off:");
    while (!Serial.available());
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "off") {
      power_on = false;
      if (send_command_with_retry(0x06180F81, power_off_cmd, power_off_resp)) {
        Serial.println("Power OFF successful.");
        step = STEP_SELECT_MODE;
      } else {
        Serial.println("Failed to power OFF after retries.");
      }
    } else {
      Serial.println("Invalid input. Type 'off' to power off.");
    }
  } else {
    Serial.println("Failed to power ON after retries.");
  }
}

// Setup function
void setup() {
  Serial.begin(115200);
  setup_twai();
  xTaskCreatePinnedToCore(repeat_command_task, "RepeatCommand", 2048, NULL, 1, NULL, 0);
}

// Main loop
void loop() {
  switch (step) {
    case STEP_SELECT_MODE:  handleModeSelection(); break;
    case STEP_SET_MODE:     handleSetMode(); break;
    case STEP_INPUT_PHASE:  handlePhaseVoltage(); break;
    case STEP_SET_PHASE:    handleSetPhase(); break;
    case STEP_POWER_CTRL:   handlePowerControl(); break;
    case STEP_POWER_ON:     handlePowerOnOff(); break;
  }
}
