#include "lkmotor.hpp"

namespace devices {

LkMotor::LkMotor(Config config, Model model) : Actuator(config), model_(model) {
  max_current = 2000;
}

void LkMotor::on_can_rx(const uint8_t* data, uint8_t len) {
  if (data == nullptr || len < 8) {
    return;
  }
  update_sensor_data(data);
}

void LkMotor::set_output() { current_set = 0; }

}  // namespace devices
