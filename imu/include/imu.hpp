#pragma once

#include "pid.hpp"
#include "frequency_filter.hpp"

namespace devices {

struct AccData {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float sensor_time = 0.0f;
  float temperature = 0.0f;
};

struct GyroData {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct SelfTest {
  bool acc_chip_id_err = true;
  bool acc_data_err = true;
  bool gyro_chip_id_err = true;
  bool gyro_data_err = true;
  bool init_err = true;
  bool calibrate_err = false;
  bool temp_ctrl_err = true;
};

class Imu {
 public:
  AccData acc_data_{};
  GyroData gyro_data_{};
  SelfTest self_test_{};

  pid::Pid temp_pid_{0.1f, 0.0f, 0.0f, 10.0f, 3.0f, pid::Position};
  float target_temp_ = 0.0f;
  filter::IirFilter temp_filter_{2.0f, 0.707f, 0.001f};

  virtual ~Imu() = default;

  virtual void config() = 0;
  virtual void calibrate() = 0;
  virtual void read_acc_data(AccData* data) = 0;
  virtual void read_gyro_data(GyroData* data) = 0;
  virtual void read_acc_temperature(float* temp) = 0;
  virtual void verify_acc_chip_id() = 0;
  virtual void verify_gyro_chip_id() = 0;
  virtual void verify_acc_data() = 0;
  virtual void verify_gyro_data() = 0;
  virtual void temperature_control(float target_temp) = 0;
};

}  // namespace devices
