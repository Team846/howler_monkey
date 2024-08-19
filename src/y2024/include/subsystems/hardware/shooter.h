#pragma once

#include <frc/AnalogInput.h>
#include <frc/AnalogTrigger.h>
#include <frc/filter/SlewRateLimiter.h>

#include "frc846/base/loggable.h"
#include "frc846/control/control.h"
#include "frc846/control/motion.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/util/grapher.h"
#include "frc846/util/pref.h"
#include "ports.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

struct ShooterReadings {
  double error_percent;
};

enum ShooterState { kRun, kIdle };

struct ShooterTarget {
  ShooterState target_state;
};

class ShooterSubsystem
    : public frc846::robot::GenericSubsystem<ShooterReadings, ShooterTarget> {
 public:
  ShooterSubsystem(bool init);

  void Setup() override;

  ShooterTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  frc846::Pref<units::feet_per_second_t> shooter_speed_{*this, "shooter_speed",
                                                        65.0_fps};

  frc846::Pref<double> spin_{*this, "shooter_spin", 0.33};

  frc846::Pref<double> shooting_exit_velocity_{*this, "shooting_exit_velocity",
                                               47.0};

  frc846::Pref<double> shooter_speed_tolerance_{
      *this, "shooter_speed_tolerance", 0.05};

 private:
  frc846::base::Loggable readings_named_{*this, "readings"};
  frc846::Grapher<double> readings_shooting_speed_left_graph{
      readings_named_, "readings_shooting_speed_left_graph"};
  frc846::Grapher<double> readings_shooting_speed_right_graph{
      readings_named_, "readings_shooting_speed_right_graph"};

  frc846::Grapher<double> shooter_right_error_graph_{
      readings_named_, "shooter_right_error_graph"};
  frc846::Grapher<double> shooter_left_error_graph_{readings_named_,
                                                    "shooter_left_error_graph"};

  frc846::base::Loggable target_named_{*this, "target"};
  frc846::Grapher<bool> target_is_shooting_graph{target_named_,
                                                 "target_is_shooting_graph"};
  frc846::Grapher<double> target_shooting_speed_graph{
      target_named_, "target_shooting_speed_graph"};

  frc846::control::ConfigHelper config_helper_{
      *this,
      {false,
       (4.75 / 12.0) * 3.14159265,
       frc846::control::MotorIdleMode::kDefaultCoast,
       {40_A}},
      {0.05, 0.00, 0.15}};

  frc846::control::HardLimitsConfigHelper<units::foot_t> hard_limits_{
      *this, {0_ft, 0_ft, false, 1.0, -1.0}};

  frc846::control::REVSparkController<units::foot_t> shooter_esc_one_{
      *this, ports::scorer_::kShooterOneController_CANID, config_helper_,
      hard_limits_};
  frc846::control::REVSparkController<units::foot_t> shooter_esc_two_{
      *this, ports::scorer_::kShooterTwoController_CANID, config_helper_,
      hard_limits_};

  ShooterReadings ReadFromHardware() override;

  void WriteToHardware(ShooterTarget target) override;

  frc846::base::Loggable vFPID_loggable{*this, "vFPID"};

  frc846::motion::BrakingVelocityFPID<units::feet_per_second_t> braking_v_FPID{
      vFPID_loggable,
      {35_A, frc846::control::DefaultSpecifications::stall_current_vortex,
       0.3}};
};
