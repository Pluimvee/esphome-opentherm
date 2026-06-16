/*
 * ISR-based OpenTherm protocol implementation.
 *
 * Receive: GPIO CHANGE edge interrupt + micros() timing (Melnyk-style).
 * Send:    Blocking Manchester bit-bang with delayMicroseconds(500).
 *
 * No hardware GP-timer is allocated.  The original ESPHome opentherm component
 * used a 5 kHz timer for both sampling during receive and driving the output
 * during send.  This version replaces both with simpler, hardware-timer-free
 * equivalents that work on any platform supported by ESPHome.
 *
 * IMPORTANT: use sync_mode: true in the hub.  The blocking send (~34 ms) is
 * acceptable in sync mode; in async mode it would stall the ESPHome loop.
 */

#include "opentherm.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#endif
#ifdef USE_ESP8266
#include <Arduino.h>
#endif

#include <cinttypes>
#include <string>

namespace esphome {
namespace opentherm {

using std::string;

static const char *const TAG = "opentherm_isr";

// Module-level instance pointer used by the static GPIO ISR helper.
static OpenTherm *ot_instance_ = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

// ── Data helpers (identical to original) ─────────────────────────────────────

float OpenthermData::f88() { return ((float) this->s16()) / 256.0f; }
void OpenthermData::f88(float value) { this->s16((int16_t) (value * 256)); }

uint16_t OpenthermData::u16() {
  uint16_t const value = this->valueHB;
  return (value << 8) | this->valueLB;
}
void OpenthermData::u16(uint16_t value) {
  this->valueLB = value & 0xFF;
  this->valueHB = (value >> 8) & 0xFF;
}

int16_t OpenthermData::s16() {
  int16_t const value = this->valueHB;
  return (value << 8) | this->valueLB;
}
void OpenthermData::s16(int16_t value) {
  this->valueLB = value & 0xFF;
  this->valueHB = (value >> 8) & 0xFF;
}

// ── Constructor ───────────────────────────────────────────────────────────────

OpenTherm::OpenTherm(InternalGPIOPin *in_pin, InternalGPIOPin *out_pin, int32_t device_timeout)
    : in_pin_(in_pin),
      out_pin_(out_pin),
      mode_(OperationMode::IDLE),
      error_type_(ProtocolErrorType::NO_ERROR),
      device_timeout_(device_timeout) {
  this->isr_in_pin_ = in_pin->to_isr();
  this->isr_out_pin_ = out_pin->to_isr();
}

// ── Initialization ────────────────────────────────────────────────────────────

bool OpenTherm::initialize() {
  this->in_pin_->pin_mode(gpio::FLAG_INPUT);
  this->in_pin_->setup();
  this->out_pin_->pin_mode(gpio::FLAG_OUTPUT);
  this->out_pin_->setup();
  this->out_pin_->digital_write(true);  // idle HIGH

  ot_instance_ = this;

#ifdef USE_ESP32
  // Install GPIO ISR service (no-op if already installed by ESPHome core).
  esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "Failed to install GPIO ISR service: %d", err);
    return false;
  }
  const auto pin_num = (gpio_num_t) this->in_pin_->get_pin();
  gpio_set_intr_type(pin_num, GPIO_INTR_ANYEDGE);
  gpio_isr_handler_add(pin_num, handle_interrupt_helper_, this);
  gpio_intr_disable(pin_num);  // enabled only when listen() is called
#endif

  return true;
}

// ── GPIO edge ISR ─────────────────────────────────────────────────────────────
//
// Called on every CHANGE of the input pin.  Implements Melnyk-style Manchester
// decoding: only edges >750 µs from the last processed edge represent new bits.
//
// OpenTherm Manchester convention on the input line (after transceiver):
//   idle  = LOW
//   "1" bit: HIGH (first half) → LOW (second half); mid-bit edge is HIGH→LOW
//   "0" bit: LOW  (first half) → HIGH (second half); mid-bit edge is LOW→HIGH
//
// At a significant (>750 µs) edge: bit value = !level_after_edge
//   H→L edge → !LOW  = 1 ✓
//   L→H edge → !HIGH = 0 ✓

void IRAM_ATTR OpenTherm::handle_interrupt_helper_(void *arg) {
  auto *self = static_cast<OpenTherm *>(arg);
  self->handle_interrupt_();
}

void IRAM_ATTR OpenTherm::handle_interrupt_() {
  const uint32_t now = micros();
  const bool level = this->isr_in_pin_.digital_read();

  if (this->mode_ == OperationMode::LISTEN) {
    if (level) {
      // Rising edge: first half of start bit.
      this->start_bit_seen_ = true;
      this->last_edge_us_ = now;
    } else if (this->start_bit_seen_ && (now - this->last_edge_us_) < 750) {
      // Falling edge within 750 µs: start bit confirmed, begin data read.
      this->mode_ = OperationMode::READ;
      this->data_ = 0;
      this->bit_pos_ = 0;
      this->last_edge_us_ = now;
      this->start_bit_seen_ = false;
    } else {
      this->start_bit_seen_ = false;  // false start, reset
    }
    return;
  }

  if (this->mode_ == OperationMode::READ) {
    const uint32_t dt = now - this->last_edge_us_;
    if (dt < 750)
      return;  // mid-bit transition, skip

    this->last_edge_us_ = now;

    if (this->bit_pos_ < 32) {
      this->data_ = (this->data_ << 1) | static_cast<uint32_t>(!level);
      this->bit_pos_++;
      return;
    }

    // Stop bit received — update mode only, do NOT call gpio_intr_disable from ISR context.
    // The interrupt stays enabled; subsequent edges are ignored (mode is no longer READ/LISTEN).
    if (!level) {  // HIGH→LOW: logical "1" stop bit
      if (check_parity_(this->data_)) {
        this->mode_ = OperationMode::RECEIVED;
      } else {
        this->error_type_ = ProtocolErrorType::PARITY_ERROR;
        this->error_data_ = this->data_;
        this->error_bit_pos_ = this->bit_pos_;
        this->mode_ = OperationMode::ERROR_PROTOCOL;
      }
    } else {  // LOW→HIGH: invalid stop bit
      this->error_type_ = ProtocolErrorType::INVALID_STOP_BIT;
      this->error_data_ = this->data_;
      this->error_bit_pos_ = this->bit_pos_;
      this->mode_ = OperationMode::ERROR_PROTOCOL;
    }
  }
}

// ── Interrupt attach / detach ─────────────────────────────────────────────────

void OpenTherm::listen() {
  this->detach_interrupt_();  // ensure clean state
  this->mode_ = OperationMode::LISTEN;
  this->data_ = 0;
  this->bit_pos_ = 0;
  this->start_bit_seen_ = false;
  this->listen_start_ms_ = millis();

#ifdef USE_ESP32
  gpio_intr_enable((gpio_num_t) this->in_pin_->get_pin());
  // If the line is already HIGH we missed the rising edge of the start bit.
  // Set start_bit_seen_ so the next falling edge will correctly enter READ mode.
  if (this->isr_in_pin_.digital_read()) {
    this->start_bit_seen_ = true;
    this->last_edge_us_ = micros();
  }
#elif defined(USE_ESP8266)
  attachInterrupt(digitalPinToInterrupt(this->in_pin_->get_pin()),
                  []() IRAM_ATTR { ot_instance_->handle_interrupt_(); }, CHANGE);
#endif
}

void OpenTherm::detach_interrupt_() {
#ifdef USE_ESP32
  gpio_intr_disable((gpio_num_t) this->in_pin_->get_pin());
#elif defined(USE_ESP8266)
  detachInterrupt(digitalPinToInterrupt(this->in_pin_->get_pin()));
#endif
}

void OpenTherm::stop() {
  this->detach_interrupt_();
  this->mode_ = OperationMode::IDLE;
}

// ── Send: blocking Manchester bit-bang ───────────────────────────────────────
//
// Each bit period = 1 ms = two 500 µs half-periods.
// Logical 1: first half LOW  (active), second half HIGH (idle)
// Logical 0: first half HIGH (idle),   second half LOW  (active)
// (OpenTherm output active = LOW)
//
// FreeRTOS tasks are suspended during the 34 ms frame so that task-switch
// jitter does not corrupt the Manchester timing.

void OpenTherm::send_bit_(bool high) {
  if (high) {
    this->isr_out_pin_.digital_write(false);  // first half: active (LOW)
    delayMicroseconds(500);
    this->isr_out_pin_.digital_write(true);   // second half: idle (HIGH)
  } else {
    this->isr_out_pin_.digital_write(true);   // first half: idle (HIGH)
    delayMicroseconds(500);
    this->isr_out_pin_.digital_write(false);  // second half: active (LOW)
  }
  delayMicroseconds(500);
}

void OpenTherm::send(OpenthermData &data) {
  this->mode_ = OperationMode::WRITE;

  uint32_t frame = data.type;
  frame = (frame << 12) | data.id;
  frame = (frame << 8) | data.valueHB;
  frame = (frame << 8) | data.valueLB;
  if (!check_parity_(frame))
    frame |= 0x80000000UL;

#ifdef USE_ESP32
  // Suspend FreeRTOS task switching to keep 500 µs timing tight.
  vTaskSuspendAll();
#endif

  this->send_bit_(true);   // start bit (always "1")
  for (int i = 31; i >= 0; i--)
    this->send_bit_(read_bit(frame, i));
  this->send_bit_(true);   // stop bit  (always "1")

  this->isr_out_pin_.digital_write(true);  // return to idle

#ifdef USE_ESP32
  xTaskResumeAll();
#endif

  this->mode_ = OperationMode::SENT;
}

// ── Response retrieval ────────────────────────────────────────────────────────

bool OpenTherm::get_message(OpenthermData &data) {
  if (this->mode_ != OperationMode::RECEIVED)
    return false;
  data.type    = (this->data_ >> 28) & 0x7;
  data.id      = (this->data_ >> 16) & 0xFF;
  data.valueHB = (this->data_ >> 8)  & 0xFF;
  data.valueLB = this->data_ & 0xFF;
  return true;
}

bool OpenTherm::get_protocol_error(OpenThermError &error) {
  if (this->mode_ != OperationMode::ERROR_PROTOCOL)
    return false;
  error.error_type = this->error_type_;
  error.data       = this->error_data_;
  error.bit_pos    = this->error_bit_pos_;
  error.capture    = 0;
  error.clock      = 0;
  return true;
}

// ── Parity ────────────────────────────────────────────────────────────────────

bool OpenTherm::check_parity_(uint32_t val) {
  val ^= val >> 16;
  val ^= val >> 8;
  val ^= val >> 4;
  val ^= val >> 2;
  val ^= val >> 1;
  return (~val) & 1;
}

// ── Logging helpers (identical to original) ───────────────────────────────────

void OpenTherm::debug_data(OpenthermData &data) {
  char type_buf[9], id_buf[9], hb_buf[9], lb_buf[9];
  ESP_LOGD(TAG, "%s %s %s %s",
           format_bin_to(type_buf, data.type),
           format_bin_to(id_buf,   data.id),
           format_bin_to(hb_buf,   data.valueHB),
           format_bin_to(lb_buf,   data.valueLB));
  ESP_LOGD(TAG, "type: %s; id: %u; HB: %u; LB: %u; uint_16: %u; float: %f",
           this->message_type_to_str((MessageType) data.type),
           data.id, data.valueHB, data.valueLB, data.u16(), data.f88());
}

void OpenTherm::debug_error(OpenThermError &error) const {
  ESP_LOGD(TAG, "data: 0x%08" PRIx32 "; bit_pos: %u", error.data, error.bit_pos);
}

// ── String helpers ────────────────────────────────────────────────────────────

#define TO_STRING_MEMBER(name) \
  case name: \
    return #name;

const char *OpenTherm::operation_mode_to_str(OperationMode mode) {
  switch (mode) {
    TO_STRING_MEMBER(IDLE)
    TO_STRING_MEMBER(LISTEN)
    TO_STRING_MEMBER(READ)
    TO_STRING_MEMBER(RECEIVED)
    TO_STRING_MEMBER(WRITE)
    TO_STRING_MEMBER(SENT)
    TO_STRING_MEMBER(ERROR_PROTOCOL)
    TO_STRING_MEMBER(ERROR_TIMEOUT)
    TO_STRING_MEMBER(ERROR_TIMER)
    default: return "<INVALID>";
  }
}

const char *OpenTherm::protocol_error_to_str(ProtocolErrorType error_type) {
  switch (error_type) {
    TO_STRING_MEMBER(NO_ERROR)
    TO_STRING_MEMBER(NO_TRANSITION)
    TO_STRING_MEMBER(INVALID_STOP_BIT)
    TO_STRING_MEMBER(PARITY_ERROR)
    TO_STRING_MEMBER(NO_CHANGE_TOO_LONG)
    default: return "<INVALID>";
  }
}

const char *OpenTherm::timer_error_to_str(TimerErrorType error_type) {
  switch (error_type) {
    TO_STRING_MEMBER(NO_TIMER_ERROR)
    TO_STRING_MEMBER(SET_ALARM_VALUE_ERROR)
    TO_STRING_MEMBER(TIMER_START_ERROR)
    TO_STRING_MEMBER(TIMER_PAUSE_ERROR)
    TO_STRING_MEMBER(SET_COUNTER_VALUE_ERROR)
    default: return "<INVALID>";
  }
}

const char *OpenTherm::message_type_to_str(MessageType message_type) {
  switch (message_type) {
    TO_STRING_MEMBER(READ_DATA)
    TO_STRING_MEMBER(READ_ACK)
    TO_STRING_MEMBER(WRITE_DATA)
    TO_STRING_MEMBER(WRITE_ACK)
    TO_STRING_MEMBER(INVALID_DATA)
    TO_STRING_MEMBER(DATA_INVALID)
    TO_STRING_MEMBER(UNKNOWN_DATAID)
    default: return "<INVALID>";
  }
}

const char *OpenTherm::message_id_to_str(MessageId id) {
  switch (id) {
    TO_STRING_MEMBER(STATUS)
    TO_STRING_MEMBER(CH_SETPOINT)
    TO_STRING_MEMBER(CONTROLLER_CONFIG)
    TO_STRING_MEMBER(DEVICE_CONFIG)
    TO_STRING_MEMBER(COMMAND_CODE)
    TO_STRING_MEMBER(FAULT_FLAGS)
    TO_STRING_MEMBER(REMOTE)
    TO_STRING_MEMBER(COOLING_CONTROL)
    TO_STRING_MEMBER(CH2_SETPOINT)
    TO_STRING_MEMBER(CH_SETPOINT_OVERRIDE)
    TO_STRING_MEMBER(TSP_COUNT)
    TO_STRING_MEMBER(TSP_COMMAND)
    TO_STRING_MEMBER(FHB_SIZE)
    TO_STRING_MEMBER(FHB_COMMAND)
    TO_STRING_MEMBER(MAX_MODULATION_LEVEL)
    TO_STRING_MEMBER(MAX_BOILER_CAPACITY)
    TO_STRING_MEMBER(ROOM_SETPOINT)
    TO_STRING_MEMBER(MODULATION_LEVEL)
    TO_STRING_MEMBER(CH_WATER_PRESSURE)
    TO_STRING_MEMBER(DHW_FLOW_RATE)
    TO_STRING_MEMBER(DAY_TIME)
    TO_STRING_MEMBER(DATE)
    TO_STRING_MEMBER(YEAR)
    TO_STRING_MEMBER(ROOM_SETPOINT_CH2)
    TO_STRING_MEMBER(ROOM_TEMP)
    TO_STRING_MEMBER(FEED_TEMP)
    TO_STRING_MEMBER(DHW_TEMP)
    TO_STRING_MEMBER(OUTSIDE_TEMP)
    TO_STRING_MEMBER(RETURN_WATER_TEMP)
    TO_STRING_MEMBER(SOLAR_STORE_TEMP)
    TO_STRING_MEMBER(SOLAR_COLLECT_TEMP)
    TO_STRING_MEMBER(FEED_TEMP_CH2)
    TO_STRING_MEMBER(DHW2_TEMP)
    TO_STRING_MEMBER(EXHAUST_TEMP)
    TO_STRING_MEMBER(FAN_SPEED)
    TO_STRING_MEMBER(FLAME_CURRENT)
    TO_STRING_MEMBER(ROOM_TEMP_CH2)
    TO_STRING_MEMBER(REL_HUMIDITY)
    TO_STRING_MEMBER(DHW_BOUNDS)
    TO_STRING_MEMBER(CH_BOUNDS)
    TO_STRING_MEMBER(OTC_CURVE_BOUNDS)
    TO_STRING_MEMBER(DHW_SETPOINT)
    TO_STRING_MEMBER(MAX_CH_SETPOINT)
    TO_STRING_MEMBER(OTC_CURVE_RATIO)
    TO_STRING_MEMBER(HVAC_STATUS)
    TO_STRING_MEMBER(REL_VENT_SETPOINT)
    TO_STRING_MEMBER(DEVICE_VENT)
    TO_STRING_MEMBER(HVAC_VER_ID)
    TO_STRING_MEMBER(REL_VENTILATION)
    TO_STRING_MEMBER(REL_HUMID_EXHAUST)
    TO_STRING_MEMBER(EXHAUST_CO2)
    TO_STRING_MEMBER(SUPPLY_INLET_TEMP)
    TO_STRING_MEMBER(SUPPLY_OUTLET_TEMP)
    TO_STRING_MEMBER(EXHAUST_INLET_TEMP)
    TO_STRING_MEMBER(EXHAUST_OUTLET_TEMP)
    TO_STRING_MEMBER(EXHAUST_FAN_SPEED)
    TO_STRING_MEMBER(SUPPLY_FAN_SPEED)
    TO_STRING_MEMBER(REMOTE_VENTILATION_PARAM)
    TO_STRING_MEMBER(NOM_REL_VENTILATION)
    TO_STRING_MEMBER(HVAC_NUM_TSP)
    TO_STRING_MEMBER(HVAC_IDX_TSP)
    TO_STRING_MEMBER(HVAC_FHB_SIZE)
    TO_STRING_MEMBER(HVAC_FHB_IDX)
    TO_STRING_MEMBER(RF_SIGNAL)
    TO_STRING_MEMBER(DHW_MODE)
    TO_STRING_MEMBER(OVERRIDE_FUNC)
    TO_STRING_MEMBER(SOLAR_MODE_FLAGS)
    TO_STRING_MEMBER(SOLAR_ASF)
    TO_STRING_MEMBER(SOLAR_VERSION_ID)
    TO_STRING_MEMBER(SOLAR_PRODUCT_ID)
    TO_STRING_MEMBER(SOLAR_NUM_TSP)
    TO_STRING_MEMBER(SOLAR_IDX_TSP)
    TO_STRING_MEMBER(SOLAR_FHB_SIZE)
    TO_STRING_MEMBER(SOLAR_FHB_IDX)
    TO_STRING_MEMBER(SOLAR_STARTS)
    TO_STRING_MEMBER(SOLAR_HOURS)
    TO_STRING_MEMBER(SOLAR_ENERGY)
    TO_STRING_MEMBER(SOLAR_TOTAL_ENERGY)
    TO_STRING_MEMBER(FAILED_BURNER_STARTS)
    TO_STRING_MEMBER(BURNER_FLAME_LOW)
    TO_STRING_MEMBER(OEM_DIAGNOSTIC)
    TO_STRING_MEMBER(BURNER_STARTS)
    TO_STRING_MEMBER(CH_PUMP_STARTS)
    TO_STRING_MEMBER(DHW_PUMP_STARTS)
    TO_STRING_MEMBER(DHW_BURNER_STARTS)
    TO_STRING_MEMBER(BURNER_HOURS)
    TO_STRING_MEMBER(CH_PUMP_HOURS)
    TO_STRING_MEMBER(DHW_PUMP_HOURS)
    TO_STRING_MEMBER(DHW_BURNER_HOURS)
    TO_STRING_MEMBER(OT_VERSION_CONTROLLER)
    TO_STRING_MEMBER(OT_VERSION_DEVICE)
    TO_STRING_MEMBER(VERSION_CONTROLLER)
    TO_STRING_MEMBER(VERSION_DEVICE)
    default: return "<INVALID>";
  }
}

}  // namespace opentherm
}  // namespace esphome
