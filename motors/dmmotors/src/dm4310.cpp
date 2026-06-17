#include "dm4310.hpp"

#include "numeric.hpp"

#include <cstring>

namespace devices {

namespace {

constexpr float kPMax = 12.56637061f;
constexpr float kPMin = -12.56637061f;
constexpr float kVMax = 15.0f;
constexpr float kVMin = -15.0f;
constexpr float kTMax = 10.0f;
constexpr float kTMin = -10.0f;

}  // namespace

Dm4310::Dm4310(const Config& config) : DmMotor(config) {
  lower_pos_limit_ = kPMin;
  upper_pos_limit_ = kPMax;
}

float Dm4310::p_max() const { return kPMax; }
float Dm4310::p_min() const { return kPMin; }
float Dm4310::v_max() const { return kVMax; }
float Dm4310::v_min() const { return kVMin; }
float Dm4310::t_max() const { return kTMax; }
float Dm4310::t_min() const { return kTMin; }

void Dm4310::set_output() {
  uint8_t output[8] = {0};
  uint32_t frame_id = can_id();

  position_set = numeric::float_constrain(position_set, lower_pos_limit_, upper_pos_limit_);

  switch (control_mode) {
    case ActuatorMode::Relax:
      std::memcpy(output, kDisableFrame, 8);
      break;
    case ActuatorMode::Mit: {
      const float position =
          numeric::loop_float_constrain(position_set + offset, -numeric::kPi, numeric::kPi);
      const uint16_t pos_tmp =
          static_cast<uint16_t>(numeric::float_to_uint(position, p_min(), p_max(), 16));
      const uint16_t vel_tmp =
          static_cast<uint16_t>(numeric::float_to_uint(speed_set, v_min(), v_max(), 12));
      const uint16_t kp_tmp =
          static_cast<uint16_t>(numeric::float_to_uint(kp, kKpMin, kKpMax, 12));
      const uint16_t kd_tmp =
          static_cast<uint16_t>(numeric::float_to_uint(kd, kKdMin, kKdMax, 12));
      const uint16_t tor_tmp =
          static_cast<uint16_t>(numeric::float_to_uint(torque_set, t_min(), t_max(), 12));
      output[0] = static_cast<uint8_t>(pos_tmp >> 8);
      output[1] = static_cast<uint8_t>(pos_tmp);
      output[2] = static_cast<uint8_t>(vel_tmp >> 4);
      output[3] = static_cast<uint8_t>(((vel_tmp & 0xF) << 4) | (kp_tmp >> 8));
      output[4] = static_cast<uint8_t>(kp_tmp);
      output[5] = static_cast<uint8_t>(kd_tmp >> 4);
      output[6] = static_cast<uint8_t>(((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
      output[7] = static_cast<uint8_t>(tor_tmp);
      break;
    }
    case ActuatorMode::PosSpeed:
      frame_id += 0x100;
      std::memcpy(output, &position_set, 4);
      std::memcpy(output + 4, &speed_set, 4);
      break;
    case ActuatorMode::Speed:
      frame_id += 0x200;
      std::memcpy(output, &speed_set, 4);
      break;
    default:
      std::memcpy(output, kDisableFrame, 8);
      break;
  }

  transmit_frame(frame_id, output, 8);
}

}  // namespace devices
