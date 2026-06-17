#pragma once

#include "lkmotor.hpp"

#include <cstdint>

namespace devices {

constexpr uint32_t kControlId = 0x280;

class LkMotorHandler {
 public:
  static LkMotorHandler& instance();

  void register_motor(LkMotor* motor);
  void send_control_data();
  void update_feedback(bsp::CanBus bus, const uint8_t* rx_data, int index);

 private:
  LkMotorHandler();

  static void on_can_rx(bsp::CanBus bus, uint32_t id, const uint8_t* data, uint8_t len,
                        void* user_data);
  void ensure_bus_ready(bsp::CanBus bus, bsp::CanType type);

  LkMotor* motor_list_[3][4]{};
  uint8_t send_data_[3][8]{};
  bool rx_registered_[3]{};
};

}  // namespace devices
