#pragma once

#include "frc846/ntinf/pref.h"
#include "frc846/robot/GenericRobotContainer.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/driver.h"
#include "subsystems/abstract/gpd.h"
#include "subsystems/abstract/operator.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/abstract/vision.h"
#include "subsystems/hardware/bracer.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/leds.h"
#include "subsystems/hardware/pivot.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/hardware/telescope.h"
#include "subsystems/hardware/wrist.h"

class RobotContainer : public frc846::robot::GenericRobotContainer {
 public:
  frc846::ntinf::Pref<bool> init_drivetrain_{*this, "init_drivetrain", true};
  frc846::ntinf::Pref<bool> init_intake_{*this, "init_intake", true};
  frc846::ntinf::Pref<bool> init_shooter_{*this, "init_shooter", true};
  frc846::ntinf::Pref<bool> init_wrist_{*this, "init_wrist", true};
  frc846::ntinf::Pref<bool> init_pivot_{*this, "init_pivot", true};
  frc846::ntinf::Pref<bool> init_telescope_{*this, "init_telescope", true};
  frc846::ntinf::Pref<bool> init_leds_{*this, "init_leds", true};
  frc846::ntinf::Pref<bool> init_bracer_{*this, "init_bracers_", false};
  frc846::ntinf::Pref<bool> init_vision_{*this, "init_vision_", true};
  frc846::ntinf::Pref<bool> init_gpd_{*this, "init_gpd_", false};

  ControlInputSubsystem control_input_;
  DrivetrainSubsystem drivetrain_{init_drivetrain_.value()};
  IntakeSubsystem intake_{init_intake_.value()};
  ShooterSubsystem shooter_{init_shooter_.value()};
  WristSubsystem wrist_{init_wrist_.value()};
  PivotSubsystem pivot_{init_pivot_.value()};
  TelescopeSubsystem telescope_{init_telescope_.value()};
  BracerSubsystem bracer_{init_bracer_.value()};
  LEDsSubsystem leds_{init_leds_.value()};
  SuperStructureSubsystem super_structure_{&pivot_, &wrist_, &telescope_};
  VisionSubsystem vision_{init_vision_.value()};
  GPDSubsystem gpd_{init_gpd_.value()};

  RobotContainer() {
    RegisterSubsystems({&control_input_, &drivetrain_, &intake_, &shooter_,
                        &wrist_, &pivot_, &telescope_, &bracer_, &leds_,
                        &super_structure_, &vision_, &gpd_});
  }
};
