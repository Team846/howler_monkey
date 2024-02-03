#ifndef FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_
#define FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_

#include "frc846/pref.h"
#include "subsystems/driver.h"
#include "subsystems/drivetrain.h"
#include "subsystems/operator.h"
#include "shintake.h"
#include "arm.h"

class RobotContainer : public frc846::Loggable {
 public:
  RobotContainer() : frc846::Loggable{"robot_container"} {}

  frc846::Pref<bool> init_drivetrain_{*this, "init_drivetrain", true};
  frc846::Pref<bool> init_shintake_{*this, "init_shintake", true};
  frc846::Pref<bool> init_arm_{*this, "init_arm", true};

  DriverSubsystem driver_;
  OperatorSubsystem operator_;
  DrivetrainSubsystem drivetrain_{init_drivetrain_.value()};
  ShintakeSubsystem shintake_{init_shintake_.value()};
  ArmSubsystem arm_{init_arm_.value()};

  std::vector<frc846::SubsystemBase*> all_subsystems_{
      &driver_,  &operator_, &drivetrain_, &shintake_, &arm_};
};

#endif  // FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_