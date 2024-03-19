#ifndef y2024_SUBSYSTEMS_BRACER_H_
#define y2024_SUBSYSTEMS_BRACER_H_


#include "frc846/util/grapher.h"
#include "frc846/loggable.h"
#include "frc846/util/pref.h"
#include "frc846/subsystem.h"
#include "ports.h"
#include "frc846/control/control.h"
#include "frc846/control/controlgains.h"
#include "units/length.h"
#include "frc/Servo.h"
#include "frc/PWM.h"
#include "frc/motorcontrol/Spark.h"
#include "frc/DigitalInput.h"

enum BracerState {
  kExtend, kStow, kRetract
};

struct BracerReadings {
};

struct BracerTarget {
  BracerState state;
};


class BracerSubsystem
    : public frc846::Subsystem<BracerReadings, BracerTarget> {
 public:
  BracerSubsystem(bool init);

  BracerTarget ZeroTarget() const override;

  BracerTarget MakeTarget(BracerState extend);

  bool VerifyHardware() override;

 private:
  int counter = 75;

  frc846::Grapher<bool> left_climb_{*this, "left_hook_engaged"};
  frc846::Grapher<bool> right_climb_{*this, "right_hook_engaged"};

  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<bool> target_extend_{target_named_, "extend"};
  
  // frc::Servo bracer_{ports::bracer_::kPWM_Left};
  // frc::Servo bracer_right_{ports::bracer_::kPWM_Right};

  frc::Spark bracer_{ports::bracer_::kPWM_Left};
  // frc::Spark bracer_right_{ports::bracer_::kPWM_Right};

  frc::DigitalInput left_switch_{3};
  frc::DigitalInput right_switch_{4};

  BracerState lastState = BracerState::kStow;

  BracerReadings GetNewReadings() override;

  void DirectWrite(BracerTarget target) override;
};

#endif