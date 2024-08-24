#pragma once

#include "frc846/base/loggable.h"
#include "frc846/control/control.h"
#include "frc846/ntinf/grapher.h"
#include "frc846/ntinf/pref.h"
#include "frc846/robot/GenericSubsystem.h"
#include "ports.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

struct IntakeReadings {};

enum IntakeState { kIntake, kPull, kFeed, kRelease, kHold };

struct IntakeTarget {
  IntakeState target_state;
};

class IntakeSubsystem
    : public frc846::robot::GenericSubsystem<IntakeReadings, IntakeTarget> {
 public:
  IntakeSubsystem(bool init);

  void Setup() override;

  IntakeTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool GetHasPiece() { return has_piece_; }

  frc846::ntinf::Pref<units::feet_per_second_t> base_intake_speed_{
      *this, "base_intake_speed_", 2_fps};

  frc846::ntinf::Pref<units::feet_per_second_t> intake_feed_speed_{
      *this, "intake_feed_speed_", 2_fps};

  frc846::ntinf::Pref<double> release_speed_{*this, "release_speed", -0.3};

  frc846::ntinf::Pref<double> retract_speed_{*this, "retract_speed", -0.2};

 private:
  bool has_piece_;

  units::feet_per_second_t target_intaking_speed;

  frc846::base::Loggable readings_named_{*this, "readings"};
  frc846::ntinf::Grapher<bool> readings_has_piece_graph{readings_named_,
                                                        "readings_has_piece"};

  frc846::ntinf::Grapher<double> readings_intake_speed_{
      readings_named_, "readings_intake_speed"};

  frc846::ntinf::Grapher<double> intake_error_{readings_named_, "intake_error"};
  frc846::ntinf::Grapher<double> intake_current_draw_{readings_named_,
                                                      "intake_current_draw"};

  frc846::base::Loggable target_named_{*this, "target"};
  frc846::ntinf::Grapher<bool> target_is_intaking_graph{
      target_named_, "target_is_intaking_graph"};

  frc846::control::ConfigHelper config_helper_{
      *this,
      {false,
       (1.5 / 12.0) * 3.14159265 * 15.0 / 50.0,
       frc846::control::MotorIdleMode::kDefaultBrake,
       {40_A}},
      {0.05, 0.00, 0.15}};

  frc846::control::HardLimitsConfigHelper<units::foot_t> hard_limits_{
      *this, {0_ft, 0_ft, false, 1.0, -1.0}};

  frc846::control::REVSparkController<units::foot_t> intake_esc_{
      *this, ports::scorer_::kController_CANID, config_helper_, hard_limits_};

  std::optional<rev::SparkLimitSwitch> in_limit_switch;
  std::optional<rev::SparkLimitSwitch> out_limit_switch;

  IntakeReadings ReadFromHardware() override;

  void WriteToHardware(IntakeTarget target) override;
};
