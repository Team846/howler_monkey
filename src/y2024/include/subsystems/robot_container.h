#ifndef FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_
#define FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_

#include "frc846/util/pref.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/driver.h"
#include "subsystems/abstract/operator.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/abstract/vision.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/leds.h"
#include "subsystems/hardware/pivot.h"
//#include "subsystems/hardware/shooter.h"
#include "subsystems/hardware/telescope.h"
#include "subsystems/hardware/wrist.h"

class RobotContainer : public frc846::Loggable {
 public:
  RobotContainer() : frc846::Loggable{"robot_container"} {}

  frc846::Pref<bool> init_drivetrain_{*this, "init_drivetrain", true};
  frc846::Pref<bool> init_intake_{*this, "init_intake", true};
  //frc846::Pref<bool> init_shooter_{*this, "init_shooter", true};
  frc846::Pref<bool> init_wrist_{*this, "init_wrist", true};
  frc846::Pref<bool> init_pivot_{*this, "init_pivot", true};
  frc846::Pref<bool> init_telescope_{*this, "init_telescope", true};
  frc846::Pref<bool> init_leds_{*this, "init_leds", true};
  frc846::Pref<bool> init_vision_{*this, "init_vision_", true};

  DriverSubsystem driver_;
  OperatorSubsystem operator_;
  ControlInputSubsystem control_input_;
  DrivetrainSubsystem drivetrain_{init_drivetrain_.value()};
  IntakeSubsystem intake_{init_intake_.value()};
  
  WristSubsystem wrist_{init_wrist_.value()};
  PivotSubsystem pivot_{init_pivot_.value()};
  TelescopeSubsystem telescope_{init_telescope_.value()};
  LEDsSubsystem leds_{init_leds_.value()};
  SuperStructureSubsystem super_structure_{&pivot_, &wrist_, &telescope_};
  VisionSubsystem vision_{init_vision_.value()};

  std::vector<frc846::SubsystemBase*> all_subsystems_{
      &driver_,  &operator_,        &control_input_, &drivetrain_, &intake_,
      &wrist_,           &pivot_,         &telescope_,  &leds_,
      &super_structure_, &vision_};
};

#endif  // FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_