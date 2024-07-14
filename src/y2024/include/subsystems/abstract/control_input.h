#pragma once

#include "frc846/subsystem.h"
#include "subsystems/abstract/driver.h"
#include "subsystems/abstract/operator.h"

struct ControlInputReadings {
  int stageOfTrap;

  bool running_intake;
  bool running_source;
  bool running_pass;
  bool running_prep_shoot;
  bool running_super_shoot;
  bool running_amp;

  bool shooting;

  bool manual_spin_up;
  bool manual_feed;
  bool eject;

  double pivot_manual_adjust;
  double telescope_manual_adjust;
  double wrist_manual_adjust;

  bool amping_leds;
  bool coopertition_leds;
};

struct ControlInputTarget {};

class ControlInputSubsystem
    : public frc846::Subsystem<ControlInputReadings, ControlInputTarget> {
 public:
  ControlInputSubsystem();

  void Setup() override {};

  ControlInputTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void UpdateWithInput(ControlInputReadings newReadings);

 private:
  ControlInputReadings this_readings_{0,     false, false, false, false, false,
                                      false, false, false, false, false, 0,
                                      0,     0,     false, false};
  ControlInputReadings previous_readings_{0,     false, false, false, false, false,
                                    false, false, false, false, false, 0,
                                    0,     0,     false, false};

  ControlInputReadings GetNewReadings() override;

  void DirectWrite(ControlInputTarget target) override;
};
