#ifndef FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_
#define FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_

#include "frc846/util/pref.h"
#include "subsystems/driver.h"
#include "subsystems/drivetrain.h"
#include "subsystems/operator.h"
#include "scorer.h"
#include "wrist.h"
#include "pivot.h"
#include "telescope.h"
#include "leds.h"

class RobotContainer : public frc846::Loggable {
 public:
  RobotContainer() : frc846::Loggable{"robot_container"} {}

  frc846::Pref<bool> init_drivetrain_{*this, "init_drivetrain", true};
  frc846::Pref<bool> init_scorer_{*this, "init_scorer", true};
  frc846::Pref<bool> init_wrist_{*this, "init_wrist", true};
  frc846::Pref<bool> init_pivot_{*this, "init_pivot", true};
  frc846::Pref<bool> init_telescope_{*this, "init_telescope", true};
  frc846::Pref<bool> init_leds_{*this, "init_leds", true};

  DriverSubsystem driver_;
  OperatorSubsystem operator_;
  DrivetrainSubsystem drivetrain_{false};
  ScorerSubsystem scorer_{false}; //init_scorer_.value()};
  WristSubsystem wrist_{init_wrist_.value()};
  PivotSubsystem pivot_{false}; //init_pivot_.value()};
  TelescopeSubsystem telescope_{false}; //init_telescope_.value()};
  LEDsSubsystem leds_{init_leds_.value()};

  std::vector<frc846::SubsystemBase*> all_subsystems_{
      &driver_,  &operator_, &drivetrain_, &scorer_, &wrist_, &pivot_, &telescope_, &leds_};
};

#endif  // FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_