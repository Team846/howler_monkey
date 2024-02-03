#ifndef y2024_SUBSYSTEMS_SCORER_H_
#define y2024_SUBSYSTEMS_SCORER_H_

#include "frcLib846/grapher.h"
#include "frcLib846/loggable.h"
#include "frcLib846/pref.h"
#include "frcLib846/subsystem.h"
#include "ports.h"
#include "frcLib846/control/control.h"
#include "frcLib846/control/newgains.h"
#include "units/angular_velocity.h"
#include "frc/AnalogInput.h"
#include "frc/AnalogTrigger.h"


struct ScorerReadings {
  
};

struct ScorerTarget {
  bool run_intake;
  bool run_shooter;
  units::turns_per_second_t shooter_speed;
};


class ScorerSubsystem
    : public frcLib846::Subsystem<ScorerReadings, ScorerTarget> {
 public:
  ScorerSubsystem(bool init);

  ScorerTarget ZeroTarget() const override;
  ScorerTarget MakeTarget(bool intake, bool shoot, units::turns_per_second_t shoot_spd);

  bool VerifyHardware() override;

  bool GetHasPiece() { return has_piece_; }

  frcLib846::Pref<double> intake_speed_{*this, "intake_speed_", 0.3};
  frcLib846::Pref<double> intake_feed_speed_{*this, "intake_feed_speed_", 0.1};
  frcLib846::Pref<units::turns_per_second_t> shooter_speed_{*this, "shooter_speed_", 50_tps};

 private:
  bool has_piece_;

  frcLib846::Loggable readings_named_{*this, "readings"};
  frcLib846::Grapher<bool> readings_has_piece_graph{readings_named_, "readings_has_piece"};

  frcLib846::Loggable target_named_{*this, "target"};
  frcLib846::Grapher<bool> target_is_intaking_graph{target_named_, "target_is_intaking_graph"};
  frcLib846::Grapher<bool> target_is_shooting_graph{target_named_, "target_is_shooting_graph"};
  frcLib846::Grapher<double> target_shooting_speed_graph{target_named_, "target_shooting_speed_graph"};


  frcLib846::Loggable intake_gains_lg = frcLib846::Loggable(*this, "intake_gains");
  frcLib846::control::ControlGainsHelper intake_esc_gains_{intake_gains_lg,
    {0, 0, 0, 0, 0}, 
        100_A, 0.5};

  frcLib846::Loggable shooter_gains_lg = frcLib846::Loggable(*this, "shooter_gains");
  frcLib846::control::ControlGainsHelper shooter_esc_gains_{shooter_gains_lg,
    {0, 0, 0, 0, 0}, 
        200_A, 0.5};

  frcLib846::control::SparkRevController<units::turn_t> intake_shooter_esc_{*this, 
    "intake_shooter_esc_", ports::scorer_::kController_CANID};


  frc::AnalogTrigger note_detector_{0};


  ScorerReadings GetNewReadings() override;

  void DirectWrite(ScorerTarget target) override;
};

#endif