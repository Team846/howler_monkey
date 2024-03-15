#ifndef y2024_SUBSYSTEMS_SCORER_H_
#define y2024_SUBSYSTEMS_SCORER_H_

#include "frc846/util/grapher.h"
#include "frc846/loggable.h"
#include "frc846/util/pref.h"
#include "frc846/subsystem.h"
#include "ports.h"
#include "frc846/control/control.h"
#include "frc846/control/controlgains.h"
#include "units/angular_velocity.h"
#include "frc/AnalogInput.h"
#include "frc/AnalogTrigger.h"
#include "frc/filter/SlewRateLimiter.h"


struct ScorerReadings {
  double kLeftErrorPercent;
};

enum ScorerState {
  kIdle, kIntake, kSpinUp, kShoot, kRelease, kScorerTest
};

struct ScorerTarget {
  ScorerState target_state;
  double intake_dc;
  double shooter_one_dc;
  double shooter_two_dc;
};


class ScorerSubsystem
    : public frc846::Subsystem<ScorerReadings, ScorerTarget> {
 public:
  ScorerSubsystem(bool init);

  ScorerTarget ZeroTarget() const override;
  ScorerTarget MakeTarget(ScorerState target_state, double intake_dc=0.0, double shooter_one_dc=0.0, double shooter_two_dc=0.0);

  bool VerifyHardware() override;

  bool GetHasPiece() { return has_piece_; }

  frc846::Pref<units::turns_per_second_t> intake_speed_{*this, "intake_speed_", 70_tps};
  // frc846::Pref<double> intake_feed_speed_{*this, "intake_feed_speed_dc_", 0.7};

  frc846::Pref<units::turns_per_second_t> intake_feed_speed_{*this, "intake_feed_speed_v_", 50_tps};
  frc846::Pref<units::turns_per_second_t> shooter_speed_{*this, "shooter_speed_", -50_tps};

  frc846::Pref<double> spin_{*this, "shooter_spin", 0.33};

  frc846::Pref<double> release_speed_{*this, "release_speed", -0.3};

  frc846::Pref<double> shooting_exit_velocity_{*this, "shooting_exit_velocity", 47.0};

  frc846::Loggable service_loggable{*this, "service_mode"};
  frc846::Pref<double> service_intake_forward_dc{service_loggable, "intake_forward", 0.0};
  frc846::Pref<double> service_intake_backward_dc{service_loggable, "intake_backward", 0.0};
  frc846::Pref<double> service_shooter_forward_dc{service_loggable, "shooter_forward", 0.0};
  frc846::Pref<double> service_shooter_backward_dc{service_loggable, "shooter_backward", 0.0};


 private:
  bool has_piece_;

  frc846::Loggable readings_named_{*this, "readings"};
  frc846::Grapher<bool> readings_has_piece_graph{readings_named_, "readings_has_piece"};
  frc846::Grapher<double> readings_shooting_speed_left_graph{readings_named_, "readings_shooting_speed_left_graph"};
  frc846::Grapher<double> readings_intake_speed_{readings_named_, "readings_intake_speed"};

  frc846::Loggable target_named_{*this, "target"};
  frc846::Grapher<bool> target_is_intaking_graph{target_named_, "target_is_intaking_graph"};
  frc846::Grapher<bool> target_is_shooting_graph{target_named_, "target_is_shooting_graph"};
  frc846::Grapher<double> target_shooting_speed_graph{target_named_, "target_shooting_speed_graph"};


  frc846::Loggable intake_gains_lg = frc846::Loggable(*this, "intake_gains");
  frc846::control::ControlGainsHelper intake_esc_gains_{intake_gains_lg,
    {0, 0, 0, 0, 0}, 
        100_A, 0.5};

  frc846::Loggable shooter_gains_lg = frc846::Loggable(*this, "shooter_gains");
  frc846::control::ControlGainsHelper shooter_esc_gains_{shooter_gains_lg,
    {0, 0, 0, 0, 0}, 
        200_A, 0.5};


  frc846::control::SparkRevController<units::turn_t> intake_shooter_esc_{*this, 
    "intake_shooter_esc_", ports::scorer_::kController_CANID};

  frc846::control::SparkFlexController<units::turn_t> shooter_esc_one_{*this, 
    "shooter_esc_one_", ports::scorer_::kShooterOneController_CANID};

  frc846::control::SparkFlexController<units::turn_t> shooter_esc_two_{*this, 
    "shooter_esc_two", ports::scorer_::kShooterTwoController_CANID};

  rev::SparkLimitSwitch note_detection;
  rev::SparkLimitSwitch note_detection_other;

  ScorerReadings GetNewReadings() override;

  void DirectWrite(ScorerTarget target) override;
};

#endif