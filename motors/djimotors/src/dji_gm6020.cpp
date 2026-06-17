#include "dji_gm6020.hpp"

namespace devices {

DjiGm6020::DjiGm6020(const Config& config) : DjiMotor(config) {
  gear_box_ = GearBox::None;
  control_mode = ActuatorMode::Relax;
}

}  // namespace devices
