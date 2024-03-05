// #include "commands/wait_till_spin_up.h"

// #include <frc/RobotBase.h>

// #include <cmath>

// #include "frc846/util/math.h"
// #include "frc846/wpilib/time.h"

// #include "subsystems/field.h"

// #include "frc846/util/share_tables.h"

// WaitTillSpinUpCommand::WaitTillSpinUpCommand(
//     RobotContainer& container)
//     : frc846::Loggable{"wait_till_spin_up_command"}, 
//           scorer_(container.scorer_) {
//   AddRequirements({});
//   SetName("wait_till_spin_up");
// }

// void WaitTillSpinUpCommand::Initialize() {
//   Log("Wait Till Spin Up Initialize");
// }

// void WaitTillSpinUpCommand::Execute() {

// }

// void WaitTillSpinUpCommand::End(bool interrupted) {
//   Log("Wait Till Spin Up Finished");
// }

// bool WaitTillSpinUpCommand::IsFinished() {
//   return std::abs(scorer_.readings().kLeftErrorPercent) < 0.15;
// }