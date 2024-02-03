#include "frcLib846/wpilib/time.h"

namespace frcLib846::wpilib {

units::second_t CurrentFPGATime() {
  int err;
  return units::microsecond_t(HAL_GetFPGATime(&err));
}

}  // namespace frcLib846::wpilib