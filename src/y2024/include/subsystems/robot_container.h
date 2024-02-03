#ifndef frcLib846_SUBSYSTEMS_ROBOT_CONTAINER_H_
#define frcLib846_SUBSYSTEMS_ROBOT_CONTAINER_H_

#include "frcLib846/pref.h"
#include "subsystems/driver.h"
#include "subsystems/drivetrain.h"
#include "subsystems/operator.h"
#include "scorer.h"
#include "scoring_positioner.h"

class RobotContainer : public frcLib846::Loggable {
 public:
  RobotContainer() : frcLib846::Loggable{"robot_container"} {}

  frcLib846::Pref<bool> init_drivetrain_{*this, "init_drivetrain", true};
  frcLib846::Pref<bool> init_scorer_{*this, "init_scorer", true};
  frcLib846::Pref<bool> init_scoring_positioner_{*this, "init_scoring_positioner", true};

  DriverSubsystem driver_;
  OperatorSubsystem operator_;
  DrivetrainSubsystem drivetrain_{init_drivetrain_.value()};
  ScorerSubsystem scorer_{init_scorer_.value()};
  ScoringPositionerSubsystem scoring_positioner_{init_scoring_positioner_.value()};

  std::vector<frcLib846::SubsystemBase*> all_subsystems_{
      &driver_,  &operator_, &drivetrain_, &scorer_, &scoring_positioner_};
};

#endif  // frcLib846_SUBSYSTEMS_ROBOT_CONTAINER_H_