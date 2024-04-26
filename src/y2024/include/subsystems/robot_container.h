#ifndef FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_
#define FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_

#include "frc846/util/pref.h"
#include "subsystems/bracer.h"
#include "subsystems/control_input.h"
#include "subsystems/driver.h"
#include "subsystems/drivetrain.h"
#include "subsystems/leds.h"
#include "subsystems/operator.h"
#include "subsystems/pivot.h"
#include "subsystems/scorer.h"
#include "subsystems/super_structure.h"
#include "subsystems/telescope.h"
#include "subsystems/vision.h"
#include "subsystems/wrist.h"

class RobotContainer : public frc846::Loggable {
 public:
  RobotContainer() : frc846::Loggable{"robot_container"} {}

  frc846::Pref<bool> init_drivetrain_{*this, "init_drivetrain", true};
  frc846::Pref<bool> init_scorer_{*this, "init_scorer", true};
  frc846::Pref<bool> init_wrist_{*this, "init_wrist", true};
  frc846::Pref<bool> init_pivot_{*this, "init_pivot", true};
  frc846::Pref<bool> init_telescope_{*this, "init_telescope", true};
  frc846::Pref<bool> init_leds_{*this, "init_leds", true};
  frc846::Pref<bool> init_bracer_{*this, "init_bracers_",
                                  true};  // add bracer command FIX
  frc846::Pref<bool> init_vision_{*this, "init_vision_", true};

  DriverSubsystem driver_;
  OperatorSubsystem operator_;
  ControlInputSubsystem control_input_;
  SuperStructureSubsystem super_structure_{true};
  DrivetrainSubsystem drivetrain_{init_drivetrain_.value()};
  ScorerSubsystem scorer_{init_scorer_.value()};
  WristSubsystem wrist_{init_wrist_.value()};
  PivotSubsystem pivot_{init_pivot_.value()};
  TelescopeSubsystem telescope_{init_telescope_.value()};
  BracerSubsystem bracer_{init_bracer_.value()};
  LEDsSubsystem leds_{init_leds_.value()};
  VisionSubsystem vision_{init_vision_.value()};

  std::vector<frc846::SubsystemBase*> all_subsystems_{
      &driver_, &operator_, &control_input_,   &drivetrain_,
      &scorer_, &wrist_,    &pivot_,           &telescope_,
      &leds_,   &bracer_,   &super_structure_, &vision_};
};

#endif  // FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_