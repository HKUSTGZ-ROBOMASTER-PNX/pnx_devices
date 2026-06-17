#pragma once

#include "lkmotor.hpp"

namespace devices {

class Lk8016 : public LkMotor {
 public:
  explicit Lk8016(const Config& config);

  static constexpr float kTorqueConstant = 0.0230399882f;
  static constexpr float kTorqueToCurrent = 43.4028f;
  static constexpr float kCurrentToTorque = kTorqueConstant;
  static constexpr float kGearRatio = 0.166667f;

  void set_output() override;
  void update_sensor_data(const uint8_t* buffer) override;
};

class Lk8025 : public LkMotor {
 public:
  explicit Lk8025(const Config& config);

  static constexpr float kTorqueConstant = 0.00512f;
  static constexpr float kTorqueToCurrent = 195.3125f;
  static constexpr float kCurrentToTorque = kTorqueConstant;

  void set_output() override;
  void update_sensor_data(const uint8_t* buffer) override;
};

}  // namespace devices
