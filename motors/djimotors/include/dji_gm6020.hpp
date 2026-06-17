#pragma once

#include "djimotor.hpp"

namespace devices {

class DjiGm6020 : public DjiMotor {
 public:
  explicit DjiGm6020(const Config& config);

  static constexpr float kTorqueConstant = 0.0f;
  static constexpr float kTorqueToCurrent = 50.0f;
  static constexpr float kCurrentToTorque = 0.02f;
};

MOTOR_DEFINE_TRAITS(DjiGm6020, 0.0f, 50.0f, 0.02f);

}  // namespace devices
