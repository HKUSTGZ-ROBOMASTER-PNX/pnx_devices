#include "dm8009p.hpp"

#include "numeric.hpp"

#include <cstring>

namespace devices {

namespace {

constexpr float kPMax = 12.5f;
constexpr float kPMin = -12.5f;
constexpr float kVMax = 45.0f;
constexpr float kVMin = -45.0f;
constexpr float kTMax = 54.0f;
constexpr float kTMin = -54.0f;

}  // namespace

Dm8009p::Dm8009p(const Config& config) : DmMotor(config) {
  lower_pos_limit_ = kPMin;
  upper_pos_limit_ = kPMax;
}

float Dm8009p::p_max() const { return kPMax; }
float Dm8009p::p_min() const { return kPMin; }
float Dm8009p::v_max() const { return kVMax; }
float Dm8009p::v_min() const { return kVMin; }
float Dm8009p::t_max() const { return kTMax; }
float Dm8009p::t_min() const { return kTMin; }

void Dm8009p::set_output() {
  uint8_t output[8] = {0};
  uint32_t frame_id = can_id();

  switch (control_mode) {
    case ActuatorMode::Relax:
      std::memcpy(output, kDisableFrame, 8);
      break;
    case ActuatorMode::Mit: {
      const uint16_t pos_tmp =
          static_cast<uint16_t>(numeric::float_to_uint(0.0f, p_min(), p_max(), 16));
      const uint16_t vel_tmp =
          static_cast<uint16_t>(numeric::float_to_uint(0.0f, v_min(), v_max(), 12));
      const uint16_t kp_tmp =
          static_cast<uint16_t>(numeric::float_to_uint(0.0f, kKpMin, kKpMax, 12));
      const uint16_t kd_tmp =
          static_cast<uint16_t>(numeric::float_to_uint(0.0f, kKdMin, kKdMax, 12));
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
      position_set = numeric::float_constrain(position_set, lower_pos_limit_, upper_pos_limit_);
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
