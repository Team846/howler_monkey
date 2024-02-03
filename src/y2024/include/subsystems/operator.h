#ifndef y2024_SUBSYSTEMS_OPERATOR_H_
#define y2024_SUBSYSTEMS_OPERATOR_H_

#include "frcLib846/grapher.h"
#include "frcLib846/math.h"
#include "frcLib846/pref.h"
#include "frcLib846/subsystem.h"
#include "frcLib846/xbox.h"
#include "ports.h"

using OperatorReadings = frcLib846::XboxReadings;

struct OperatorTarget {
  bool rumble;
};

class OperatorSubsystem
    : public frcLib846::Subsystem<OperatorReadings, OperatorTarget> {
 public:
  OperatorSubsystem();

  OperatorTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frcLib846::Pref<double> trigger_threshold_{*this, "trigger_threshold", 0.3};
  frcLib846::Pref<double> rumble_strength_{*this, "rumble_strength", 1.0};

  frc::XboxController xbox_{ports::operator_::kXbox_DSPort};

  frcLib846::Loggable target_loggable_{*this, "target"};
  frcLib846::Grapher<bool> target_rumble_graph_{target_loggable_, "rumble"};

  OperatorReadings GetNewReadings() override;

  void DirectWrite(OperatorTarget target) override;
};

#endif  // y2024_SUBSYSTEMS_Dawg_H_