#pragma once

#include "frc/DigitalInput.h"
#include "frc/PWM.h"
#include "frc/Servo.h"
#include "frc/motorcontrol/Spark.h"
#include "frc846/base/loggable.h"
#include "frc846/control/control.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/util/grapher.h"
#include "frc846/util/pref.h"
#include "ports.h"
#include "units/length.h"

enum BracerState { kStow, kExtend, kRetract };

struct BracerReadings {};

struct BracerTarget {
  BracerState state;
};

class BracerSubsystem
    : public frc846::robot::GenericSubsystem<BracerReadings, BracerTarget> {
 public:
  BracerSubsystem(bool init);

  void Setup() override {};

  BracerTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::Grapher<bool> left_climb_{*this, "left_hook_engaged"};
  frc846::Grapher<bool> right_climb_{*this, "right_hook_engaged"};

  frc846::base::Loggable target_named_{*this, "target"};

  frc846::Grapher<bool> target_extend_{target_named_, "extend"};

  frc::Spark bracer_{ports::bracer_::kPWM_Left};

  frc::DigitalInput left_switch_{3};
  frc::DigitalInput right_switch_{4};

  BracerReadings ReadFromHardware() override;

  void WriteToHardware(BracerTarget target) override;
};
