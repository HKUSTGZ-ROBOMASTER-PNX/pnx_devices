#include "dmmotor.hpp"

#include "bsp_can.hpp"
#include "numeric.hpp"

namespace devices {

const uint8_t DmMotor::kEnableFrame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t DmMotor::kDisableFrame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
const uint8_t DmMotor::kSaveZeroFrame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
const uint8_t DmMotor::kClearErrorFrame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

DmMotor::DmMotor(Config config) : Actuator(config) {}

State DmMotor::alive_check() {
  if (alive_flag_ == pre_alive_flag_) {
    motor_state = State::Offline;
  } else {
    pre_alive_flag_ = alive_flag_;
    motor_state = State::Online;
  }
  return motor_state;
}

void DmMotor::on_can_rx(const uint8_t* data, uint8_t len) {
  if (data == nullptr || len < 8) {
    return;
  }
  receive_data(data);
}

void DmMotor::transmit_frame(uint32_t frame_id, const uint8_t* data, uint16_t len) {
  bsp::can_transmit(bus(), frame_id, data, len);
}

void DmMotor::receive_data(const uint8_t* buffer) {
  alive_flag_++;
  raw_.id = buffer[0] & 0x0F;
  raw_.err = static_cast<ErrorCode>(buffer[0] >> 4);

  const uint16_t p_int = static_cast<uint16_t>((buffer[1] << 8) | buffer[2]);
  const uint16_t v_int = static_cast<uint16_t>((buffer[3] << 4) | (buffer[4] >> 4));
  const uint16_t t_int = static_cast<uint16_t>(((buffer[4] & 0x0F) << 8) | buffer[5]);

  feedback.position_fdb = numeric::loop_float_constrain(
      numeric::uint_to_float(static_cast<int>(p_int), p_min(), p_max(), 16), -numeric::kPi,
      numeric::kPi);
  feedback.speed_fdb = numeric::uint_to_float(static_cast<int>(v_int), v_min(), v_max(), 12);
  feedback.torque_fdb = numeric::uint_to_float(static_cast<int>(t_int), t_min(), t_max(), 12);
  raw_.temp_mos = static_cast<float>(buffer[6]);
  raw_.temp_rotor = static_cast<float>(buffer[7]);
  feedback.temperature_fdb = raw_.temp_rotor;
}

}  // namespace devices
