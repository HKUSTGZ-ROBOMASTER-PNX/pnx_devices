#include "dji_m3508.hpp"

namespace devices {

DjiM3508::DjiM3508(const Config& config) : DjiMotor(config) {
  gear_box_ = GearBox::M3508;
  control_mode = ActuatorMode::Relax;
}

}  // namespace devices
