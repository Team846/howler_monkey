#ifndef y2024_SUBSYSTEMS_DRIVER_H_
#define y2024_SUBSYSTEMS_DRIVER_H_

#include "frcLib846/grapher.h"
#include "frcLib846/math.h"
#include "frcLib846/pref.h"
#include "frcLib846/subsystem.h"
#include "frcLib846/xbox.h"
#include "ports.h"

using DriverReadings = frcLib846::XboxReadings;

struct DriverTarget {
  bool rumble;
};

class DriverSubsystem : public frcLib846::Subsystem<DriverReadings, DriverTarget> {
 public:
  DriverSubsystem();

  frcLib846::Pref<double> translation_deadband_{*this, "translation_deadband",
                                             0.05};
  frcLib846::Pref<double> steer_deadband_{*this, "steer_deadband", 0.05};

  frcLib846::Pref<int> translation_exponent_{*this, "translation_exponent", 1};
  frcLib846::Pref<int> steer_exponent_{*this, "steer_exponent", 2};

  DriverTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frcLib846::Pref<double> trigger_threshold_{*this, "trigger_threshold", 0.3};
  frcLib846::Pref<double> rumble_strength_{*this, "rumble_strength", 1.0};

  frc::XboxController xbox_{ports::driver_::kXbox_DSPort};

  frcLib846::Loggable target_loggable_{*this, "target"};
  frcLib846::Grapher<bool> target_rumble_graph_{target_loggable_, "rumble"};

  DriverReadings GetNewReadings() override;

  void DirectWrite(DriverTarget target) override;
};

#endif  // y2024_SUBSYSTEMS_DRIVER_H_