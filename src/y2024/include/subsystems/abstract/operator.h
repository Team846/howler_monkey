#pragma once

#include "frc846/other/xbox.h"
#include "frc846/subsystem.h"
#include "frc846/util/grapher.h"
#include "frc846/util/math.h"
#include "frc846/util/pref.h"
#include "ports.h"

using OperatorReadings = frc846::XboxReadings;

struct OperatorTarget {
  bool rumble;
};

class OperatorSubsystem
    : public frc846::Subsystem<OperatorReadings, OperatorTarget> {
 public:
  OperatorSubsystem();

  OperatorTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::Pref<double> trigger_threshold_{*this, "trigger_threshold", 0.3};
  frc846::Pref<double> rumble_strength_{*this, "rumble_strength", 1.0};

  frc::XboxController xbox_{ports::operator_::kXbox_DSPort};

  frc846::Loggable target_loggable_{*this, "target"};
  frc846::Grapher<bool> target_rumble_graph_{target_loggable_, "rumble"};

  OperatorReadings previous_readings_;

  OperatorReadings GetNewReadings() override;

  void DirectWrite(OperatorTarget target) override;
};
