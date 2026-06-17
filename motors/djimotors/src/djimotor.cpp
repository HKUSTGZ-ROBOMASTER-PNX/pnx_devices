#include "djimotor.hpp"

#include "numeric.hpp"

namespace devices {

namespace {

constexpr float kPos2RadNone = 0.0007669903939f;
constexpr float kPos2RadM2006 = 0.00002130788978f;
constexpr float kPos2RadM3508 = 0.00003994074176f;
constexpr float kPos2RadXRoll = 0.000048658315f;

constexpr float kRpm2RpsNone = 0.1047197551196f;
constexpr float kRpm2RpsM2006 = 0.002908882087f;
constexpr float kRpm2RpsM3508 = 0.005453242609f;
constexpr float kRpm2RpsXRoll = 0.006642670920f;

}  // namespace

DjiMotor::DjiMotor(Config config) : Actuator(config) {}

void DjiMotor::on_can_rx(const uint8_t* data, uint8_t len) {
  if (data == nullptr || len < 7) {
    return;
  }
  update_sensor_data(data);
}

float DjiMotor::rpm_to_rad_s(GearBox box, int16_t rpm) {
  switch (box) {
    case GearBox::M2006:
      return static_cast<float>(rpm) * kRpm2RpsM2006;
    case GearBox::M3508:
      return static_cast<float>(rpm) * kRpm2RpsM3508;
    case GearBox::XRoll:
      return static_cast<float>(rpm) * kRpm2RpsXRoll;
    default:
      return static_cast<float>(rpm) * kRpm2RpsNone;
  }
}

float DjiMotor::ecd_to_rad(GearBox box, const RawFeedback& raw) {
  float ratio = kPos2RadNone;
  switch (box) {
    case GearBox::M2006:
      ratio = kPos2RadM2006;
      break;
    case GearBox::M3508:
      ratio = kPos2RadM3508;
      break;
    case GearBox::XRoll:
      ratio = kPos2RadXRoll;
      break;
    default:
      break;
  }
  return static_cast<float>(raw.ecd + raw.ecd_cnt * 8192 - raw.ecd_offset) * ratio;
}

void DjiMotor::update_sensor_data(const uint8_t* can_data) {
  alive_flag_++;

  raw_.last_ecd = raw_.ecd;
  raw_.ecd = static_cast<uint16_t>((can_data[0] << 8) | can_data[1]);
  raw_.speed_rpm = static_cast<int16_t>((can_data[2] << 8) | can_data[3]);
  feedback.current_fdb =
      static_cast<float>(static_cast<int16_t>((can_data[4] << 8) | can_data[5]));
  feedback.temperature_fdb = static_cast<float>(can_data[6]);

  if (gear_box_ == GearBox::None) {
    feedback.speed_fdb = rpm_to_rad_s(gear_box_, raw_.speed_rpm);
    feedback.position_fdb =
        static_cast<float>(raw_.ecd) * kPos2RadNone - numeric::kPi;
    feedback.torque_fdb = 0.0f;
    return;
  }

  if (raw_.ecd - raw_.last_ecd > 4096) {
    raw_.ecd_cnt--;
  } else if (raw_.ecd - raw_.last_ecd < -4096) {
    raw_.ecd_cnt++;
  }
  feedback.speed_fdb = rpm_to_rad_s(gear_box_, raw_.speed_rpm);
  feedback.position_fdb = ecd_to_rad(gear_box_, raw_);
  feedback.torque_fdb = 0.0f;
}

void DjiMotor::reset_position_feedback() {
  raw_.ecd_offset = raw_.ecd;
  raw_.ecd_cnt = 0;
  raw_.total_cnt = 0;
  feedback.position_fdb = 0.0f;
}

State DjiMotor::alive_check() {
  if (alive_flag_ == pre_alive_flag_) {
    motor_state = State::Offline;
  } else {
    pre_alive_flag_ = alive_flag_;
    motor_state = State::Online;
  }
  return motor_state;
}

void DjiMotor::set_output() {
  if (control_mode == ActuatorMode::Speed) {
    speed_pid_.ref_ = speed_set;
    speed_pid_.fdb_ = feedback.speed_fdb;
    speed_pid_.update_result();
    current_set = static_cast<int16_t>(speed_pid_.result_);
  } else if (control_mode == ActuatorMode::Position) {
    position_pid_.ref_ = position_set;
    position_pid_.fdb_ = feedback.position_fdb;
    position_pid_.update_result();
    speed_set = position_pid_.result_;

    speed_pid_.ref_ = speed_set;
    speed_pid_.fdb_ = feedback.speed_fdb;
    speed_pid_.update_result();
    current_set = static_cast<int16_t>(speed_pid_.result_);
  } else {
    current_set = 0;
  }

  if (control_mode == ActuatorMode::Relax) {
    current_set = 0;
  }

  current_set = static_cast<int16_t>(
      numeric::limit_abs(static_cast<float>(current_set), static_cast<float>(max_current)));
}

}  // namespace devices
