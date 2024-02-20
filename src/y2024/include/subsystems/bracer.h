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


struct BracerReadings {
};

struct BracerTarget {
  bool extend;
};


class BracerSubsystem
    : public frc846::Subsystem<BracerReadings, BracerTarget> {
 public:
  BracerSubsystem(bool init);

  BracerTarget ZeroTarget() const override;

  BracerTarget MakeTarget(bool extend);

  bool VerifyHardware() override;

 private:
  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<bool> target_extend_{target_named_, "extend"};
  
  frc::Servo bracer_left_{ports::bracer_::kPWM_Servo_Left};
  frc::Servo bracer_right_{ports::bracer_::kPWM_Servo_Right};

  BracerReadings GetNewReadings() override;

  void DirectWrite(BracerTarget target) override;
};

#endif