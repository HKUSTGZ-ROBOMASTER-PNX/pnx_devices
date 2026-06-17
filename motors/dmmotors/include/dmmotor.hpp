#pragma once

#include "motor.hpp"
#include "pid.hpp"

#include <cstdint>

namespace devices {

constexpr float kKpMin = 0.0f;
constexpr float kKpMax = 500.0f;
constexpr float kKdMin = 0.0f;
constexpr float kKdMax = 500.0f;

enum class ErrorCode : uint8_t {
  Disable = 0x0,
  Enable = 0x1,
  OverVoltage = 0x8,
  UnderVoltage = 0x9,
  OverCurrent = 0xA,
  MosOverTemp = 0xB,
  CoilOverTemp = 0xC,
  CommLost = 0xD,
  Overload = 0xE,
};

class DmMotor : public Actuator {
 public:
  explicit DmMotor(Config config);

  Family family() const override { return Family::Dm; }
  State alive_check() override;
  void on_can_rx(const uint8_t* data, uint8_t len) override;
  virtual void set_output() = 0;

  virtual float p_max() const = 0;
  virtual float p_min() const = 0;
  virtual float v_max() const = 0;
  virtual float v_min() const = 0;
  virtual float t_max() const = 0;
  virtual float t_min() const = 0;

  void receive_data(const uint8_t* buffer);
  void transmit_frame(uint32_t frame_id, const uint8_t* data, uint16_t len);

  static const uint8_t kEnableFrame[8];
  static const uint8_t kDisableFrame[8];
  static const uint8_t kSaveZeroFrame[8];
  static const uint8_t kClearErrorFrame[8];

  uint32_t alive_flag_ = 0;
  uint32_t pre_alive_flag_ = 0;
  bool enable_failed_ = false;

  float lower_pos_limit_ = 0.0f;
  float upper_pos_limit_ = 0.0f;

  pid::Pid speed_pid_{0.1f, 0.0f, 0.0f, 16384.0f, 3.0f, pid::Position};
  pid::Pid position_pid_{0.1f, 0.0f, 0.0f, 16384.0f, 3.0f, pid::Position};

  struct RawFeedback {
    uint8_t id = 0;
    ErrorCode err = ErrorCode::Disable;
    float temp_mos = 0.0f;
    float temp_rotor = 0.0f;
  } raw_{};
};

}  // namespace devices
