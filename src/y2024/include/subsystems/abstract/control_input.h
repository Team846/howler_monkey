#pragma once

#include "frc846/subsystem.h"
#include "subsystems/abstract/driver.h"
#include "subsystems/abstract/operator.h"

struct ControlInputReadings {
  double translate_x;
  double translate_y;
  double rotation;

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

  bool home_wrist;
  bool zero_bearing;
};

struct ControlInputTarget {
  bool driver_rumble;
  bool operator_rumble;
};

class ControlInputSubsystem
    : public frc846::Subsystem<ControlInputReadings, ControlInputTarget> {
 public:
  ControlInputSubsystem();

  void Setup() override {};

  ControlInputTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  ControlInputReadings UpdateWithInput();

  DriverSubsystem driver_;
  OperatorSubsystem operator_;

 private:
  ControlInputReadings previous_readings_{};

  DriverReadings previous_driver_{};
  OperatorReadings previous_operator_{};

  ControlInputReadings GetNewReadings() override;

  void DirectWrite(ControlInputTarget target) override;
};
