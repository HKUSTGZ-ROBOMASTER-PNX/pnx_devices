#pragma once

#include "djimotor.hpp"

namespace devices {

class DjiM3508 : public DjiMotor {
 public:
  explicit DjiM3508(const Config& config);

  static constexpr float kTorqueConstant = 0.0f;
  static constexpr float kTorqueToCurrent = 1400.0f;
  static constexpr float kCurrentToTorque = 1.0f / 1400.0f;
};

MOTOR_DEFINE_TRAITS(DjiM3508, 0.0f, 1400.0f, 1.0f / 1400.0f);
/*
template <>
struct ActuatorTraits<DjiM3508> {
  static constexpr float kTorqueConstant = 0.0f;
  static constexpr float kTorqueToCurrent = 1400.0f;
  static constexpr float kCurrentToTorque = 1.0f / 1400.0f;
};
*/

}  // namespace devices
