#pragma once

#include "frc846/ntinf/grapher.h"
#include "frc846/ntinf/pref.h"
#include "frc846/other/xbox.h"
#include "frc846/robot/GenericSubsystem.h"
#include "ports.h"

using DriverReadings = frc846::XboxReadings;

struct DriverTarget {
  bool rumble;
};

class DriverSubsystem
    : public frc846::robot::GenericSubsystem<DriverReadings, DriverTarget> {
 public:
  DriverSubsystem();

  void Setup() override {};

  frc846::ntinf::Pref<double> translation_deadband_{
      *this, "translation_deadband", 0.05};
  frc846::ntinf::Pref<double> steer_deadband_{*this, "steer_deadband", 0.05};

  frc846::ntinf::Pref<int> translation_exponent_{*this, "translation_exponent",
                                                 1};
  frc846::ntinf::Pref<int> steer_exponent_{*this, "steer_exponent", 2};

  DriverTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::ntinf::Pref<double> trigger_threshold_{*this, "trigger_threshold",
                                                 0.3};
  frc846::ntinf::Pref<double> rumble_strength_{*this, "rumble_strength", 1.0};

  frc::XboxController xbox_{ports::driver_::kXbox_DSPort};

  frc846::base::Loggable target_loggable_{*this, "target"};
  frc846::ntinf::Grapher<bool> target_rumble_graph_{target_loggable_, "rumble"};

  DriverReadings previous_readings_;

  DriverReadings ReadFromHardware() override;

  void WriteToHardware(DriverTarget target) override;
};
