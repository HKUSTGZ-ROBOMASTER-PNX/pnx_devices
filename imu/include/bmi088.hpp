#pragma once

#include "imu.hpp"

#include <cstdint>

namespace devices {

enum class Bmi088Cs : uint8_t {
  Acc = 0,
  Gyro = 1,
};

class Bmi088 : public Imu {
 public:
  void config() override;
  void calibrate() override;
  void read_acc_data(AccData* data) override;
  void read_gyro_data(GyroData* data) override;
  void read_acc_temperature(float* temp) override;
  void verify_acc_chip_id() override;
  void verify_gyro_chip_id() override;
  void verify_acc_data() override;
  void verify_gyro_data() override;
  void temperature_control(float target_temp) override;

 private:
  void read_reg(Bmi088Cs cs, uint8_t addr, uint8_t* data, uint8_t len);
  void write_reg(Bmi088Cs cs, uint8_t addr, const uint8_t* data, uint8_t len);

  float gyro_offset_[3]{};
  filter::IirFilter sensor_filter_[6] = {
      filter::IirFilter(333.0f, 0.707f, 0.001f), filter::IirFilter(333.0f, 0.707f, 0.001f),
      filter::IirFilter(333.0f, 0.707f, 0.001f), filter::IirFilter(333.0f, 0.707f, 0.001f),
      filter::IirFilter(333.0f, 0.707f, 0.001f), filter::IirFilter(333.0f, 0.707f, 0.001f),
  };
};

}  // namespace devices
