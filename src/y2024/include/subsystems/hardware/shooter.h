#ifndef y2024_SUBSYSTEMS_SHOOTER_H_
#define y2024_SUBSYSTEMS_SHOOTER_H_

#include "frc/AnalogInput.h"
#include "frc/AnalogTrigger.h"
#include "frc/filter/SlewRateLimiter.h"
#include "frc846/control/control.h"
#include "frc846/control/controlgains.h"
#include "frc846/loggable.h"
#include "frc846/subsystem.h"
#include "frc846/util/grapher.h"
#include "frc846/util/pref.h"
#include "ports.h"
#include "units/angular_velocity.h"

struct ShooterReadings {
  double error_percent;
};

enum ShooterState { kRun, kIdle };

struct ShooterTarget {
  ShooterState target_state;
};

class ShooterSubsystem
    : public frc846::Subsystem<ShooterReadings, ShooterTarget> {
 public:
  ShooterSubsystem(bool init);

  ShooterTarget ZeroTarget() const override;
  ShooterTarget MakeTarget(ShooterState target_state);

  bool VerifyHardware() override;

  frc846::Pref<units::turns_per_second_t> shooter_speed_{
      *this, "shooter_speed_", -50_tps};

  frc846::Pref<double> spin_{*this, "shooter_spin", 0.33};

  frc846::Pref<double> shooting_exit_velocity_{*this, "shooting_exit_velocity",
                                               47.0};

 private:
  frc846::Loggable readings_named_{*this, "readings"};
  frc846::Grapher<double> readings_shooting_speed_left_graph{
      readings_named_, "readings_shooting_speed_left_graph"};
  frc846::Grapher<double> readings_shooting_speed_right_graph{
      readings_named_, "readings_shooting_speed_right_graph"};

  frc846::Grapher<double> shooter_right_error_graph_{
      readings_named_, "shooter_right_error_graph"};
  frc846::Grapher<double> shooter_left_error_graph_{readings_named_,
                                                    "shooter_left_error_graph"};

  frc846::Loggable target_named_{*this, "target"};
  frc846::Grapher<bool> target_is_shooting_graph{target_named_,
                                                 "target_is_shooting_graph"};
  frc846::Grapher<double> target_shooting_speed_graph{
      target_named_, "target_shooting_speed_graph"};

  frc846::Loggable shooter_gains_lg = frc846::Loggable(*this, "shooter_gains");
  frc846::control::ControlGainsHelper shooter_esc_gains_{
      shooter_gains_lg, {0, 0, 0, 0, 0}, 200_A, 0.5};

  frc846::control::SparkFlexController<units::turn_t> shooter_esc_one_{
      *this, "shooter_esc_one_", ports::scorer_::kShooterOneController_CANID};

  frc846::control::SparkFlexController<units::turn_t> shooter_esc_two_{
      *this, "shooter_esc_two", ports::scorer_::kShooterTwoController_CANID};

  ShooterReadings GetNewReadings() override;

  void DirectWrite(ShooterTarget target) override;
};

#endif