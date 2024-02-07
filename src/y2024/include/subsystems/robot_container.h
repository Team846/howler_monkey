#ifndef FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_
#define FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_

#include "frc846/pref.h"
#include "subsystems/driver.h"
#include "subsystems/drivetrain.h"
#include "subsystems/operator.h"
#include "shintake.h"
#include "wrist.h"
#include "pivot.h"
#include "telescope.h"

class RobotContainer : public frc846::Loggable {
 public:
  RobotContainer() : frc846::Loggable{"robot_container"} {}

  frc846::Pref<bool> init_drivetrain_{*this, "init_drivetrain", true};
  frc846::Pref<bool> init_shintake_{*this, "init_shintake", true};
  frc846::Pref<bool> init_wrist_{*this, "init_wrist", true};
  frc846::Pref<bool> init_pivot_{*this, "init_pivot", true};
  frc846::Pref<bool> init_telescope_{*this, "init_telescope", true};

  DriverSubsystem driver_;
  OperatorSubsystem operator_;
  DrivetrainSubsystem drivetrain_{init_drivetrain_.value()};
  ShintakeSubsystem shintake_{init_shintake_.value()};
  WristSubsystem wrist_{init_wrist_.value()};
  PivotSubsystem pivot_{init_pivot_.value()};
  TelescopeSubsystem telescope_{init_telescope_.value()};

  std::vector<frc846::SubsystemBase*> all_subsystems_{
      &driver_,  &operator_, &drivetrain_, &shintake_, &wrist_, &pivot_, &telescope_};
};

#endif  // FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_