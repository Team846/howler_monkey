#ifndef y2024_SUBSYSTEMS_SCORING_POSITIONER_H_
#define y2024_SUBSYSTEMS_SCORING_POSITIONER_H_

#include "frcLib846/grapher.h"
#include "frcLib846/loggable.h"
#include "frcLib846/pref.h"
#include "frcLib846/subsystem.h"
#include "ports.h"
#include "frcLib846/control/control.h"
#include "frcLib846/control/newgains.h"
#include "units/length.h"


struct ScoringPositionerReadings {
  units::degree_t pivot_position;
  units::degree_t wrist_position;
  units::inch_t extension;
};

struct ScoringPositionerTarget {
  std::variant<units::degree_t, double> pivot_output;
  std::variant<units::degree_t, double> wrist_output;
  std::variant<units::inch_t, double> extension;
};


class ScoringPositionerSubsystem
    : public frcLib846::Subsystem<ScoringPositionerReadings, ScoringPositionerTarget> {
 public:
  ScoringPositionerSubsystem(bool init);

  ScoringPositionerTarget ZeroTarget() const override;

  ScoringPositionerTarget MakeTarget(std::variant<units::degree_t, double> pivot_out,
    std::variant<units::degree_t, double> wrist_out, std::variant<units::inch_t, double> tele_out);

  bool VerifyHardware() override;

  bool GetHasHomed() { return hasHomed; }

  frcLib846::Pref<units::degree_t> intake_setpoint_pivot{*this, "intake_setpoint_pivot", 0_deg};
  frcLib846::Pref<units::degree_t> intake_setpoint_wrist{*this, "intake_setpoint_wrist", 0_deg};
  frcLib846::Pref<units::inch_t> intake_setpoint_tele{*this, "intake_setpoint_tele_", 0_in};

   frcLib846::Pref<units::degree_t> stow_setpoint_pivot{*this, "stow_setpoint_pivot", 0_deg};
  frcLib846::Pref<units::degree_t> stow_setpoint_wrist{*this, "stow_setpoint_wrist", 0_deg};
  frcLib846::Pref<units::inch_t> stow_setpoint_tele{*this, "stow_setpoint_tele_", 0_in};

 private:
  bool hasHomed = false;

  frcLib846::Loggable readings_named_{*this, "readings"};

  frcLib846::Grapher<double> pivot_pos_graph{target_named_, "pivot_pos"};
  frcLib846::Grapher<double> wrist_pos_graph{target_named_, "wrist_pos"};
  frcLib846::Grapher<double> tele_pos_graph{target_named_, "tele_pos"};


  frcLib846::Loggable target_named_{*this, "target"};

  frcLib846::Grapher<double> target_pivot_duty_cycle_graph{target_named_, "pivot_duty_cycle"};
  frcLib846::Grapher<double> target_wrist_duty_cycle_graph{target_named_, "wrist_duty_cycle"};
  frcLib846::Grapher<double> target_tele_duty_cycle_graph{target_named_, "tele_duty_cycle"};

  frcLib846::Grapher<double> target_pivot_pos_graph{target_named_, "pivot_pos"};
  frcLib846::Grapher<double> target_wrist_pos_graph{target_named_, "wrist_pos"};
  frcLib846::Grapher<double> target_tele_pos_graph{target_named_, "tele_pos"};


  frcLib846::Loggable pivot_gains_lg = frcLib846::Loggable(*this, "pivot_gains");
  frcLib846::control::ControlGainsHelper pivot_esc_gains_{pivot_gains_lg,
    {0, 0, 0, 0, 0}, 
        100_A, 0.5};

  frcLib846::Loggable wrist_gains_lg = frcLib846::Loggable(*this, "wrist_gains");
  frcLib846::control::ControlGainsHelper wrist_esc_gains_{wrist_gains_lg,
    {0, 0, 0, 0, 0}, 
        200_A, 0.5};

  frcLib846::Loggable tele_gains_lg = frcLib846::Loggable(*this, "tele_gains");
  frcLib846::control::ControlGainsHelper tele_esc_gains_{tele_gains_lg,
    {0, 0, 0, 0, 0}, 
        50_A, 0.5};



  frcLib846::control::SparkRevController<units::degree_t> pivot_one_{*this, 
    "pivot_one", ports::scoring_positioner_::kPivotOne_CANID};
  frcLib846::control::SparkRevController<units::degree_t> pivot_two_{*this, 
    "pivot_two", ports::scoring_positioner_::kPivotTwo_CANID};
  frcLib846::control::SparkRevController<units::degree_t> pivot_three_{*this, 
    "pivot_three", ports::scoring_positioner_::kPivotThree_CANID};
  frcLib846::control::SparkRevController<units::degree_t> pivot_four_{*this, 
    "pivot_four", ports::scoring_positioner_::kPivotFour_CANID};

  // frcLib846::control::RevParallelController<units::degree_t> pivot_tandem_{*this, "pivot", {
  //   ports::scoring_positioner_::kPivotOne_CANID,
  //   ports::scoring_positioner_::kPivotTwo_CANID,
  //   ports::scoring_positioner_::kPivotThree_CANID,
  //   ports::scoring_positioner_::kPivotFour_CANID
  // }};
  
  frcLib846::control::SparkRevController<units::degree_t> wrist_esc_{*this, 
    "wrist_esc", ports::scoring_positioner_::kWrist_CANID};

  frcLib846::control::SparkRevController<units::inch_t> tele_one_{*this, 
    "tele_one", ports::scoring_positioner_::kTele1_CANID};
  frcLib846::control::SparkRevController<units::inch_t> tele_two_{*this, 
    "tele_two", ports::scoring_positioner_::kTele2_CANID};


  // frcLib846::control::RevParallelController<units::inch_t> tele_tandem_{*this, "telescope", {
  //   ports::scoring_positioner_::kTele1_CANID,
  //   ports::scoring_positioner_::kTele2_CANID,
  // }};

  ScoringPositionerReadings GetNewReadings() override;

  void PositionTelescope(ScoringPositionerTarget target);
  void PositionPivot(ScoringPositionerTarget target);
  void PositionWrist(ScoringPositionerTarget target);

  void DirectWrite(ScoringPositionerTarget target) override;
};

#endif