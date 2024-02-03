#ifndef frcLib846_TIME_H_
#define frcLib846_TIME_H_

#include <hal/HALBase.h>
#include <units/time.h>

namespace frcLib846 {
namespace wpilib {

// Get the current time.
units::second_t CurrentFPGATime();

}  // namespace wpilib
}  // namespace frcLib846

#endif  // frcLib846_TIME_H_