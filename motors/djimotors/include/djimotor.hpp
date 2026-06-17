#pragma once

#include "motor.hpp"
#include "pid.hpp"

#include <cstdint>

namespace devices {

enum class GearBox : uint8_t {
  None = 0,
  M2006,
  M3508,
  XRoll,
};

class DjiMotor : public Actuator {
 public:
  explicit DjiMotor(Config config);

  Family family() const override { return Family::Dji; }
  void set_output() override;
  State alive_check() override;
  void on_can_rx(const uint8_t* data, uint8_t len) override;

  virtual void blocked_check() {}
  void update_sensor_data(const uint8_t* can_data);
  void reset_position_feedback();

  static constexpr float kTorqueConstant = 0.0f;
  static constexpr float kTorqueToCurrent = 0.0f;
  static constexpr float kCurrentToTorque = 0.0f;

  pid::Pid speed_pid_{0.1f, 0.0f, 0.0f, 16384.0f, 3.0f, pid::Position};
  pid::Pid position_pid_{0.1f, 0.0f, 0.0f, 25000.0f, 3.0f, pid::Position};

  uint32_t alive_flag_ = 0;
  uint32_t pre_alive_flag_ = 0;
  bool blocked_ = false;

  GearBox gear_box_ = GearBox::None;

  struct RawFeedback {
    int16_t last_ecd = 0;
    uint16_t ecd = 0;
    uint16_t ecd_offset = 0;
    int16_t speed_rpm = 0;
    int8_t ecd_cnt = 0;
    int16_t total_cnt = 0;
  } raw_{};

 private:
  static float rpm_to_rad_s(GearBox box, int16_t rpm);
  static float ecd_to_rad(GearBox box, const RawFeedback& raw);
};

}  // namespace devices
