#include "motor.hpp"

namespace devices {

void Actuator::relax() {
  control_mode = ActuatorMode::Relax;
  speed_set = 0.0f;
  position_set = 0.0f;
  torque_set = 0.0f;
  current_set = 0;
  kp = 0.0f;
  kd = 0.0f;
}

}  // namespace devices
