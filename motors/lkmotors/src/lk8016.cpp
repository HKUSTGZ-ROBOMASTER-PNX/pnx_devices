#include "lk8016.hpp"

#include "numeric.hpp"

namespace devices {

Lk8016::Lk8016(const Config& config) : LkMotor(config, Model::Lk8016) {
  control_mode = ActuatorMode::Relax;
}

void Lk8016::update_sensor_data(const uint8_t* buffer) {
  raw_.last_ecd = raw_.ecd;
  raw_.ecd = static_cast<uint16_t>((buffer[7] << 8) | buffer[6]);
  raw_.speed_dps = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);
  raw_.current_raw = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
  feedback.temperature_fdb = static_cast<float>(buffer[1]);

  feedback.position_fdb = numeric::loop_float_constrain(
      static_cast<float>(raw_.ecd - static_cast<uint16_t>(offset)) * kRawPos2Rad, -numeric::kPi,
      numeric::kPi);
  feedback.speed_fdb =
      static_cast<float>(raw_.speed_dps) * kRawDps2Rps * kGearRatio;
  feedback.current_fdb = static_cast<float>(raw_.current_raw);
  feedback.torque_fdb = feedback.current_fdb * kTorqueConstant;
}

void Lk8016::set_output() {
  if (control_mode == ActuatorMode::Relax) {
    current_set = 0;
    return;
  }
  if (control_mode == ActuatorMode::Torque) {
    current_set = static_cast<int16_t>(torque_set * kTorqueToCurrent);
    current_set = static_cast<int16_t>(numeric::float_constrain(
        static_cast<float>(current_set), -static_cast<float>(max_current),
        static_cast<float>(max_current)));
    return;
  }
  if (control_mode == ActuatorMode::Position) {
    if (position_set < -numeric::kPi) {
      position_set += 2.0f * numeric::kPi;
    } else if (position_set > numeric::kPi) {
      position_set -= 2.0f * numeric::kPi;
    }
    position_pid_.ref_ = position_set;
    position_pid_.fdb_ = feedback.position_fdb;
    position_pid_.update_result();

    speed_pid_.ref_ = position_pid_.result_;
    speed_pid_.fdb_ = feedback.speed_fdb;
    speed_pid_.update_result();
    current_set = static_cast<int16_t>(speed_pid_.result_);
  } else {
    current_set = 0;
    return;
  }

  current_set = static_cast<int16_t>(numeric::float_constrain(
      static_cast<float>(current_set), -static_cast<float>(max_current),
      static_cast<float>(max_current)));
}

}  // namespace devices
