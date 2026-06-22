#include "bmi088.hpp"

#include "bsp_dwt.hpp"
#include "bsp_pwm.hpp"
#include "numeric.hpp"
#include "os.hpp"

#include "main.h"
#include "spi.h"

#include <cmath>
#include <cstring>

namespace devices {
namespace {

constexpr uint8_t kSpiWriteCode = 0x7F;
constexpr uint8_t kSpiReadCode = 0x80;

constexpr uint8_t kAccChipIdAddr = 0x00;
constexpr uint8_t kAccChipIdVal = 0x1E;
constexpr uint8_t kAccXLsbAddr = 0x12;
constexpr uint8_t kAccXyzLen = 6;
constexpr uint8_t kAccRange6G = 0x01;
constexpr uint8_t kAccConfAddr = 0x40;
constexpr uint8_t kAccRangeAddr = 0x41;
constexpr uint8_t kInt1IoCtrlAddr = 0x53;
constexpr uint8_t kIntMapDataAddr = 0x58;
constexpr uint8_t kAccPwrConfAddr = 0x7C;
constexpr uint8_t kAccPwrConfAct = 0x00;
constexpr uint8_t kAccPwrCtrlAddr = 0x7D;
constexpr uint8_t kAccPwrCtrlOn = 0x04;
constexpr uint8_t kAccSoftResetAddr = 0x7E;
constexpr uint8_t kAccSoftResetVal = 0xB6;

constexpr uint8_t kGyroChipIdAddr = 0x00;
constexpr uint8_t kGyroChipIdVal = 0x0F;
constexpr uint8_t kGyroRateXLsbAddr = 0x02;
constexpr uint8_t kGyroXyzLen = 6;
constexpr uint8_t kGyroRangeAddr = 0x0F;
constexpr uint8_t kGyroRange1000 = 0x01;
constexpr uint8_t kGyroBandwidthAddr = 0x10;
constexpr uint8_t kGyroOdr2000Bw230 = 0x01;
constexpr uint8_t kGyroLpm1Addr = 0x11;
constexpr uint8_t kGyroLpm1Nor = 0x00;
constexpr uint8_t kGyroLpm1Sus = 0x80;
constexpr uint8_t kGyroIntCtrlAddr = 0x15;
constexpr uint8_t kGyroDrdyOn = 0x80;
constexpr uint8_t kGyroIntIoConfAddr = 0x16;
constexpr uint8_t kGyroIntIoMapAddr = 0x18;
constexpr uint8_t kGyroSoftResetAddr = 0x14;
constexpr uint8_t kGyroSoftResetVal = 0xB6;

constexpr uint8_t kTempMsbAddr = 0x22;
constexpr uint8_t kTempLen = 2;
constexpr float kTempUnit = 0.125f;
constexpr float kTempBias = 23.0f;

constexpr float kAccel6GSen = 0.00179443359375f;
constexpr float kGyro1000Sen = 0.0005326322180158476f;

constexpr float kGyroPreOffsetX = -0.005280993487f;
constexpr float kGyroPreOffsetY = -0.000237223741f;
constexpr float kGyroPreOffsetZ = -0.000647540528f;
constexpr float kAccelPreOffsetX = 0.0038458286072392397f;
constexpr float kAccelPreOffsetY = 0.00647039594993548f;
constexpr float kAccelPreOffsetZ = 0.014968990490337293f;

void cs_low(Bmi088Cs cs) {
  if (cs == Bmi088Cs::Acc) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  }
}

void cs_high(Bmi088Cs cs) {
  if (cs == Bmi088Cs::Acc) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  }
}

}  // namespace

void Bmi088::write_reg(Bmi088Cs cs, uint8_t addr, const uint8_t* data, uint8_t len) {
  cs_low(cs);
  uint8_t tx_addr = static_cast<uint8_t>(addr & kSpiWriteCode);
  HAL_SPI_Transmit(&hspi2, &tx_addr, 1, 1000);
  if (len > 0 && data != nullptr) {
    HAL_SPI_Transmit(&hspi2, const_cast<uint8_t*>(data), len, 1000);
  }
  if (self_test_.init_err) {
    bsp::dwt_delay_s(0.001f);
  }
  cs_high(cs);
}

void Bmi088::read_reg(Bmi088Cs cs, uint8_t addr, uint8_t* data, uint8_t len) {
  cs_low(cs);
  uint8_t tx_addr = static_cast<uint8_t>(addr | kSpiReadCode);
  HAL_SPI_Transmit(&hspi2, &tx_addr, 1, 1000);
  HAL_SPI_Receive(&hspi2, data, len, 1000);
  cs_high(cs);
}

void Bmi088::calibrate() {
  constexpr int kCalibSamples = 4000;
  float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
  GyroData temp_gyro{};

  gyro_offset_[0] = 0.0f;
  gyro_offset_[1] = 0.0f;
  gyro_offset_[2] = 0.0f;

  for (int i = 0; i < kCalibSamples; ++i) {
    read_gyro_data(&temp_gyro);
    gyro_sum[0] += temp_gyro.x;
    gyro_sum[1] += temp_gyro.y;
    gyro_sum[2] += temp_gyro.z;
    os::thread_sleep(1);
  }

  gyro_offset_[0] = gyro_sum[0] / static_cast<float>(kCalibSamples);
  gyro_offset_[1] = gyro_sum[1] / static_cast<float>(kCalibSamples);
  gyro_offset_[2] = gyro_sum[2] / static_cast<float>(kCalibSamples);

  if (std::fabs(gyro_offset_[0]) > 0.1f || std::fabs(gyro_offset_[1]) > 0.1f ||
      std::fabs(gyro_offset_[2]) > 0.1f) {
    self_test_.calibrate_err = true;
    gyro_offset_[0] = kGyroPreOffsetX;
    gyro_offset_[1] = kGyroPreOffsetY;
    gyro_offset_[2] = kGyroPreOffsetZ;
  } else {
    self_test_.calibrate_err = false;
  }
}

void Bmi088::temperature_control(float target_temp) {
  target_temp_ = target_temp;
  temp_pid_.ref_ = target_temp_;
  temp_pid_.fdb_ = temp_filter_.update(acc_data_.temperature);
  temp_pid_.update_result();

  float duty_ratio = temp_pid_.result_ / 999.0f;
  duty_ratio = numeric::float_constrain(duty_ratio, 0.0f, 0.08f);
  bsp::pwm_set_duty(bsp::PwmInstance::Tim3_Ch4, duty_ratio);
}

void Bmi088::verify_acc_chip_id() {
  uint8_t rx[2] = {0};
  read_reg(Bmi088Cs::Acc, kAccChipIdAddr, rx, 2);
  os::thread_sleep(1);
  if (rx[1] != kAccChipIdVal) {
    self_test_.acc_chip_id_err = true;
    self_test_.init_err = true;
  } else {
    self_test_.acc_chip_id_err = false;
  }
}

void Bmi088::verify_gyro_chip_id() {
  uint8_t rx = 0;
  read_reg(Bmi088Cs::Gyro, kGyroChipIdAddr, &rx, 1);
  os::thread_sleep(1);
  if (rx != kGyroChipIdVal) {
    self_test_.gyro_chip_id_err = true;
    self_test_.init_err = true;
  } else {
    self_test_.gyro_chip_id_err = false;
  }
}

void Bmi088::verify_acc_data() {}
void Bmi088::verify_gyro_data() {}

void Bmi088::config() {
  os::thread_sleep(10);

  uint8_t tx = kAccSoftResetVal;
  write_reg(Bmi088Cs::Acc, kAccSoftResetAddr, &tx, 1);
  os::thread_sleep(100);

  tx = kAccPwrConfAct;
  write_reg(Bmi088Cs::Acc, kAccPwrConfAddr, &tx, 1);
  os::thread_sleep(10);

  tx = kAccPwrCtrlOn;
  write_reg(Bmi088Cs::Acc, kAccPwrCtrlAddr, &tx, 1);
  os::thread_sleep(10);

  tx = kAccRange6G;
  write_reg(Bmi088Cs::Acc, kAccRangeAddr, &tx, 1);
  os::thread_sleep(5);

  tx = 0xAB;
  write_reg(Bmi088Cs::Acc, kAccConfAddr, &tx, 1);
  os::thread_sleep(5);

  tx = 0x08;
  write_reg(Bmi088Cs::Acc, kInt1IoCtrlAddr, &tx, 1);
  os::thread_sleep(5);

  tx = 0x04;
  write_reg(Bmi088Cs::Acc, kIntMapDataAddr, &tx, 1);
  os::thread_sleep(5);

  tx = kGyroSoftResetVal;
  write_reg(Bmi088Cs::Gyro, kGyroSoftResetAddr, &tx, 1);
  os::thread_sleep(100);

  tx = kGyroRange1000;
  write_reg(Bmi088Cs::Gyro, kGyroRangeAddr, &tx, 1);
  os::thread_sleep(5);

  tx = static_cast<uint8_t>(kGyroOdr2000Bw230 | kGyroLpm1Sus);
  write_reg(Bmi088Cs::Gyro, kGyroBandwidthAddr, &tx, 1);
  os::thread_sleep(5);

  tx = kGyroLpm1Nor;
  write_reg(Bmi088Cs::Gyro, kGyroLpm1Addr, &tx, 1);
  os::thread_sleep(5);

  tx = kGyroDrdyOn;
  write_reg(Bmi088Cs::Gyro, kGyroIntCtrlAddr, &tx, 1);
  os::thread_sleep(5);

  tx = 0x00;
  write_reg(Bmi088Cs::Gyro, kGyroIntIoConfAddr, &tx, 1);
  os::thread_sleep(5);

  tx = 0x01;
  write_reg(Bmi088Cs::Gyro, kGyroIntIoMapAddr, &tx, 1);
  os::thread_sleep(5);
}

void Bmi088::read_acc_data(AccData* data) {
  if (data == nullptr) {
    return;
  }

  uint8_t buf[kAccXyzLen + 1] = {0};
  int16_t acc[3] = {0};
  read_reg(Bmi088Cs::Acc, kAccXLsbAddr, buf, kAccXyzLen + 1);

  acc[0] = static_cast<int16_t>((buf[2] << 8) | buf[1]);
  acc[1] = static_cast<int16_t>((buf[4] << 8) | buf[3]);
  acc[2] = static_cast<int16_t>((buf[6] << 8) | buf[5]);

  data->x = static_cast<float>(acc[0]) * kAccel6GSen + kAccelPreOffsetX;
  data->y = static_cast<float>(acc[1]) * kAccel6GSen + kAccelPreOffsetY;
  data->z = static_cast<float>(acc[2]) * kAccel6GSen + kAccelPreOffsetZ;
  acc_data_ = *data;
}

void Bmi088::read_gyro_data(GyroData* data) {
  if (data == nullptr) {
    return;
  }

  uint8_t buf[kGyroXyzLen] = {0};
  int16_t gyro[3] = {0};
  read_reg(Bmi088Cs::Gyro, kGyroRateXLsbAddr, buf, kGyroXyzLen);

  gyro[0] = static_cast<int16_t>((buf[1] << 8) | buf[0]);
  gyro[1] = static_cast<int16_t>((buf[3] << 8) | buf[2]);
  gyro[2] = static_cast<int16_t>((buf[5] << 8) | buf[4]);

  data->x = static_cast<float>(gyro[0]) * kGyro1000Sen - gyro_offset_[0];
  data->y = static_cast<float>(gyro[1]) * kGyro1000Sen - gyro_offset_[1];
  data->z = static_cast<float>(gyro[2]) * kGyro1000Sen - gyro_offset_[2];
  gyro_data_ = *data;
}

void Bmi088::read_acc_temperature(float* temp) {
  if (temp == nullptr) {
    return;
  }

  uint8_t buf[kTempLen + 1] = {0};
  read_reg(Bmi088Cs::Acc, kTempMsbAddr, buf, kTempLen + 1);
  const uint16_t temp_uint11 = static_cast<uint16_t>((buf[1] << 3) | (buf[2] >> 5));
  int16_t temp_int11 = static_cast<int16_t>(temp_uint11);
  if (temp_uint11 > 1023) {
    temp_int11 = static_cast<int16_t>(temp_uint11 - 2048);
  }
  *temp = static_cast<float>(temp_int11) * kTempUnit + kTempBias;
  acc_data_.temperature = *temp;
}

}  // namespace devices
