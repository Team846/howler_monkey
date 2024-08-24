#pragma once

#include "frc846/ntinf/grapher.h"
#include "frc846/ntinf/pref.h"
#include "frc846/other/xbox.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/util/math.h"
#include "ports.h"

using OperatorReadings = frc846::XboxReadings;

struct OperatorTarget {
  bool rumble;
};

class OperatorSubsystem
    : public frc846::robot::GenericSubsystem<OperatorReadings, OperatorTarget> {
 public:
  OperatorSubsystem();

  void Setup() override {};

  OperatorTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::ntinf::Pref<double> trigger_threshold_{*this, "trigger_threshold",
                                                 0.3};
  frc846::ntinf::Pref<double> rumble_strength_{*this, "rumble_strength", 1.0};

  frc::XboxController xbox_{ports::operator_::kXbox_DSPort};

  frc846::base::Loggable target_loggable_{*this, "target"};
  frc846::ntinf::Grapher<bool> target_rumble_graph_{target_loggable_, "rumble"};

  OperatorReadings previous_readings_;

  OperatorReadings ReadFromHardware() override;

  void WriteToHardware(OperatorTarget target) override;
};
