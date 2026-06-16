/*
 * ISR-based OpenTherm protocol implementation for ESPHome.
 *
 * Based on the original ESPHome opentherm component (esphome/components/opentherm),
 * which was in turn based on https://github.com/jpraus/arduino-opentherm.
 *
 * Key difference from the original: replaces the hardware GP-timer ISR with a
 * GPIO edge interrupt + micros() approach, inspired by Ihor Melnyk's OpenTherm
 * library (https://github.com/ihormelnyk/opentherm_library).
 *
 * Receive: GPIO CHANGE interrupt, bit decoded from edge timing with micros().
 * Send:    Blocking Manchester encoding with delayMicroseconds(500) per half-bit.
 *          Use sync_mode: true in the hub (recommended).
 *
 * No hardware timer is allocated. The ERROR_TIMER mode is kept for API
 * compatibility with hub.cpp but is never set.
 */

#pragma once

#include <string>
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace opentherm {

template<class T> constexpr T read_bit(T value, uint8_t bit) { return (value >> bit) & 0x01; }
template<class T> constexpr T set_bit(T value, uint8_t bit) { return value |= (1UL << bit); }
template<class T> constexpr T clear_bit(T value, uint8_t bit) { return value &= ~(1UL << bit); }
template<class T> constexpr T write_bit(T value, uint8_t bit, uint8_t bit_value) {
  return bit_value ? set_bit(value, bit) : clear_bit(value, bit);
}

enum OperationMode {
  IDLE = 0,
  LISTEN = 1,
  READ = 2,
  RECEIVED = 3,
  WRITE = 4,
  SENT = 5,
  ERROR_PROTOCOL = 8,
  ERROR_TIMEOUT = 9,
  ERROR_TIMER = 10,  // never set; kept for hub.cpp API compatibility
};

enum ProtocolErrorType {
  NO_ERROR = 0,
  NO_TRANSITION = 1,
  INVALID_STOP_BIT = 2,
  PARITY_ERROR = 3,
  NO_CHANGE_TOO_LONG = 4,
};

// Kept for hub.cpp API compatibility; no timer is used.
enum TimerErrorType {
  NO_TIMER_ERROR = 0,
  SET_ALARM_VALUE_ERROR = 1,
  TIMER_START_ERROR = 2,
  TIMER_PAUSE_ERROR = 3,
  SET_COUNTER_VALUE_ERROR = 4,
};

enum MessageType {
  READ_DATA = 0,
  READ_ACK = 4,
  WRITE_DATA = 1,
  WRITE_ACK = 5,
  INVALID_DATA = 2,
  DATA_INVALID = 6,
  UNKNOWN_DATAID = 7,
};

enum MessageId {
  STATUS = 0,
  CH_SETPOINT = 1,
  CONTROLLER_CONFIG = 2,
  DEVICE_CONFIG = 3,
  COMMAND_CODE = 4,
  FAULT_FLAGS = 5,
  REMOTE = 6,
  COOLING_CONTROL = 7,
  CH2_SETPOINT = 8,
  CH_SETPOINT_OVERRIDE = 9,
  TSP_COUNT = 10,
  TSP_COMMAND = 11,
  FHB_SIZE = 12,
  FHB_COMMAND = 13,
  MAX_MODULATION_LEVEL = 14,
  MAX_BOILER_CAPACITY = 15,
  ROOM_SETPOINT = 16,
  MODULATION_LEVEL = 17,
  CH_WATER_PRESSURE = 18,
  DHW_FLOW_RATE = 19,
  DAY_TIME = 20,
  DATE = 21,
  YEAR = 22,
  ROOM_SETPOINT_CH2 = 23,
  ROOM_TEMP = 24,
  FEED_TEMP = 25,
  DHW_TEMP = 26,
  OUTSIDE_TEMP = 27,
  RETURN_WATER_TEMP = 28,
  SOLAR_STORE_TEMP = 29,
  SOLAR_COLLECT_TEMP = 30,
  FEED_TEMP_CH2 = 31,
  DHW2_TEMP = 32,
  EXHAUST_TEMP = 33,
  FAN_SPEED = 35,
  FLAME_CURRENT = 36,
  ROOM_TEMP_CH2 = 37,
  REL_HUMIDITY = 38,
  DHW_BOUNDS = 48,
  CH_BOUNDS = 49,
  OTC_CURVE_BOUNDS = 50,
  DHW_SETPOINT = 56,
  MAX_CH_SETPOINT = 57,
  OTC_CURVE_RATIO = 58,
  HVAC_STATUS = 70,
  REL_VENT_SETPOINT = 71,
  DEVICE_VENT = 74,
  HVAC_VER_ID = 75,
  REL_VENTILATION = 77,
  REL_HUMID_EXHAUST = 78,
  EXHAUST_CO2 = 79,
  SUPPLY_INLET_TEMP = 80,
  SUPPLY_OUTLET_TEMP = 81,
  EXHAUST_INLET_TEMP = 82,
  EXHAUST_OUTLET_TEMP = 83,
  EXHAUST_FAN_SPEED = 84,
  SUPPLY_FAN_SPEED = 85,
  REMOTE_VENTILATION_PARAM = 86,
  NOM_REL_VENTILATION = 87,
  HVAC_NUM_TSP = 88,
  HVAC_IDX_TSP = 89,
  HVAC_FHB_SIZE = 90,
  HVAC_FHB_IDX = 91,
  RF_SIGNAL = 98,
  DHW_MODE = 99,
  OVERRIDE_FUNC = 100,
  SOLAR_MODE_FLAGS = 101,
  SOLAR_ASF = 102,
  SOLAR_VERSION_ID = 103,
  SOLAR_PRODUCT_ID = 104,
  SOLAR_NUM_TSP = 105,
  SOLAR_IDX_TSP = 106,
  SOLAR_FHB_SIZE = 107,
  SOLAR_FHB_IDX = 108,
  SOLAR_STARTS = 109,
  SOLAR_HOURS = 110,
  SOLAR_ENERGY = 111,
  SOLAR_TOTAL_ENERGY = 112,
  FAILED_BURNER_STARTS = 113,
  BURNER_FLAME_LOW = 114,
  OEM_DIAGNOSTIC = 115,
  BURNER_STARTS = 116,
  CH_PUMP_STARTS = 117,
  DHW_PUMP_STARTS = 118,
  DHW_BURNER_STARTS = 119,
  BURNER_HOURS = 120,
  CH_PUMP_HOURS = 121,
  DHW_PUMP_HOURS = 122,
  DHW_BURNER_HOURS = 123,
  OT_VERSION_CONTROLLER = 124,
  OT_VERSION_DEVICE = 125,
  VERSION_CONTROLLER = 126,
  VERSION_DEVICE = 127,
};

enum BitPositions { STOP_BIT = 33 };

struct OpenthermData {
  uint8_t type;
  uint8_t id;
  uint8_t valueHB;
  uint8_t valueLB;

  OpenthermData() : type(0), id(0), valueHB(0), valueLB(0) {}

  float f88();
  void f88(float value);
  uint16_t u16();
  void u16(uint16_t value);
  int16_t s16();
  void s16(int16_t value);
};

struct OpenThermError {
  ProtocolErrorType error_type;
  uint32_t capture;
  uint8_t clock;
  uint32_t data;
  uint8_t bit_pos;
};

class OpenTherm {
 public:
  OpenTherm(InternalGPIOPin *in_pin, InternalGPIOPin *out_pin, int32_t device_timeout = 1000);

  bool initialize();
  void listen();
  void send(OpenthermData &data);
  bool get_message(OpenthermData &data);
  bool get_protocol_error(OpenThermError &error);
  void stop();

  bool has_message() { return mode_ == OperationMode::RECEIVED; }
  bool is_sent()     { return mode_ == OperationMode::SENT; }
  bool is_idle()     { return mode_ == OperationMode::IDLE; }
  bool is_active()   { return mode_ == LISTEN || mode_ == READ || mode_ == WRITE; }
  bool is_error()    { return mode_ == ERROR_TIMEOUT || mode_ == ERROR_PROTOCOL; }
  bool is_timeout()  { return mode_ == ERROR_TIMEOUT; }
  bool is_protocol_error() { return mode_ == ERROR_PROTOCOL; }
  bool is_timer_error()    { return false; }

  // Timeout is checked lazily in get_mode() so it works without a timer in both sync and async hub modes.
  OperationMode get_mode() {
    if ((mode_ == LISTEN || mode_ == READ) && (millis() - listen_start_ms_) > (uint32_t) device_timeout_) {
      this->detach_interrupt_();
      mode_ = ERROR_TIMEOUT;
    }
    return mode_;
  }

  void report_and_reset_timer_error() {}  // no-op

  void debug_data(OpenthermData &data);
  void debug_error(OpenThermError &error) const;

  const char *operation_mode_to_str(OperationMode mode);
  const char *protocol_error_to_str(ProtocolErrorType error_type);
  const char *timer_error_to_str(TimerErrorType error_type);
  const char *message_type_to_str(MessageType message_type);
  const char *message_id_to_str(MessageId id);

  static void IRAM_ATTR handle_interrupt_helper_(void *arg);

 private:
  InternalGPIOPin *in_pin_;
  InternalGPIOPin *out_pin_;
  ISRInternalGPIOPin isr_in_pin_;
  ISRInternalGPIOPin isr_out_pin_;

  volatile OperationMode mode_{IDLE};
  volatile uint32_t data_{0};
  volatile uint8_t bit_pos_{0};

  // Edge-based receive state
  volatile uint32_t last_edge_us_{0};   // micros() of last processed (significant) edge
  volatile bool start_bit_seen_{false}; // HIGH edge of start bit detected, waiting for LOW

  uint32_t listen_start_ms_{0};
  int32_t device_timeout_;              // ms; used for timeout in get_mode()

  ProtocolErrorType error_type_{NO_ERROR};
  uint32_t error_data_{0};
  uint8_t error_bit_pos_{0};

  void IRAM_ATTR handle_interrupt_();
  void detach_interrupt_();
  void send_bit_(bool high);
  static bool check_parity_(uint32_t val);
};

}  // namespace opentherm
}  // namespace esphome
