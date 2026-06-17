#pragma once

#include "dmmotor.hpp"

namespace devices {

class Dm4310 : public DmMotor {
 public:
  explicit Dm4310(const Config& config);

  static constexpr float kTorqueConstant = 1.0f;
  static constexpr float kTorqueToCurrent = 0.0f;
  static constexpr float kCurrentToTorque = 1.0f;

  void set_output() override;
  float p_max() const override;
  float p_min() const override;
  float v_max() const override;
  float v_min() const override;
  float t_max() const override;
  float t_min() const override;
};

class Dm8009p : public DmMotor {
 public:
  explicit Dm8009p(const Config& config);

  static constexpr float kTorqueConstant = 1.0f;
  static constexpr float kTorqueToCurrent = 0.0f;
  static constexpr float kCurrentToTorque = 1.0f;

  void set_output() override;
  float p_max() const override;
  float p_min() const override;
  float v_max() const override;
  float v_min() const override;
  float t_max() const override;
  float t_min() const override;
};

}  // namespace devices
