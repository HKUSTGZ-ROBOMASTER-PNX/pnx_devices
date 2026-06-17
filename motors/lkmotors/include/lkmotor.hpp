#pragma once

#include "motor.hpp"
#include "pid.hpp"

#include <cstdint>

namespace devices {

enum class Model : uint8_t {
  Lk8016 = 0,
  Lk8025,
};

class LkMotor : public Actuator {
 public:
  LkMotor(Config config, Model model);

  Family family() const override { return Family::Lk; }
  void set_output() override;
  State alive_check() override { return State::Online; }
  void on_can_rx(const uint8_t* data, uint8_t len) override;

  virtual void update_sensor_data(const uint8_t* buffer) = 0;

  Model model_ = Model::Lk8016;

  pid::Pid speed_pid_{0.1f, 0.0f, 0.0f, 25000.0f, 3.0f, pid::Position};
  pid::Pid position_pid_{0.1f, 0.0f, 0.0f, 25000.0f, 3.0f, pid::Position};

  static constexpr float kRawPos2Rad = 0.00009587379924285f;
  static constexpr float kRawDps2Rps = 0.0174532925199433f;

  struct RawFeedback {
    int16_t last_ecd = 0;
    uint16_t ecd = 0;
    int16_t speed_dps = 0;
    int16_t current_raw = 0;
  } raw_{};
};

}  // namespace devices
