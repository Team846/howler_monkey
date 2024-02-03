#ifndef y2024_SUBSYSTEMS_ARM_H_
#define y2024_SUBSYSTEMS_ARM_H_

#include "frc846/grapher.h"
#include "frc846/loggable.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "ports.h"
#include "frc846/control/control.h"
#include "frc846/control/newgains.h"
#include "units/length.h"


struct ArmReadings {
  units::degree_t pivot_position;
  units::degree_t wrist_position;
  units::inch_t extension;
};

struct ArmTarget {
  std::variant<units::degree_t, double> pivot_output;
  std::variant<units::degree_t, double> wrist_output;
  std::variant<units::inch_t, double> extension;
};


class ArmSubsystem
    : public frc846::Subsystem<ArmReadings, ArmTarget> {
 public:
  ArmSubsystem(bool init);

  ArmTarget ZeroTarget() const override;

  ArmTarget MakeTarget(std::variant<units::degree_t, double> pivot_out,
    std::variant<units::degree_t, double> wrist_out, std::variant<units::inch_t, double> tele_out);

  bool VerifyHardware() override;

  bool GetHasHomed() { return hasHomed; }

  frc846::Pref<units::degree_t> intake_setpoint_pivot{*this, "intake_setpoint_pivot", 0_deg};
  frc846::Pref<units::degree_t> intake_setpoint_wrist{*this, "intake_setpoint_wrist", 0_deg};
  frc846::Pref<units::inch_t> intake_setpoint_tele{*this, "intake_setpoint_tele_", 0_in};

   frc846::Pref<units::degree_t> stow_setpoint_pivot{*this, "stow_setpoint_pivot", 0_deg};
  frc846::Pref<units::degree_t> stow_setpoint_wrist{*this, "stow_setpoint_wrist", 0_deg};
  frc846::Pref<units::inch_t> stow_setpoint_tele{*this, "stow_setpoint_tele_", 0_in};

 private:
  bool hasHomed = false;

  frc846::Loggable readings_named_{*this, "readings"};

  frc846::Grapher<double> pivot_pos_graph{target_named_, "pivot_pos"};
  frc846::Grapher<double> wrist_pos_graph{target_named_, "wrist_pos"};
  frc846::Grapher<double> tele_pos_graph{target_named_, "tele_pos"};


  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<double> target_pivot_duty_cycle_graph{target_named_, "pivot_duty_cycle"};
  frc846::Grapher<double> target_wrist_duty_cycle_graph{target_named_, "wrist_duty_cycle"};
  frc846::Grapher<double> target_tele_duty_cycle_graph{target_named_, "tele_duty_cycle"};

  frc846::Grapher<double> target_pivot_pos_graph{target_named_, "pivot_pos"};
  frc846::Grapher<double> target_wrist_pos_graph{target_named_, "wrist_pos"};
  frc846::Grapher<double> target_tele_pos_graph{target_named_, "tele_pos"};


  frc846::Loggable pivot_gains_lg = frc846::Loggable(*this, "pivot_gains");
  frc846::control::ControlGainsHelper pivot_esc_gains_{pivot_gains_lg,
    {0, 0, 0, 0, 0}, 
        100_A, 0.5};

  frc846::Loggable wrist_gains_lg = frc846::Loggable(*this, "wrist_gains");
  frc846::control::ControlGainsHelper wrist_esc_gains_{wrist_gains_lg,
    {0, 0, 0, 0, 0}, 
        200_A, 0.5};

  frc846::Loggable tele_gains_lg = frc846::Loggable(*this, "tele_gains");
  frc846::control::ControlGainsHelper tele_esc_gains_{tele_gains_lg,
    {0, 0, 0, 0, 0}, 
        50_A, 0.5};



  frc846::control::SparkRevController<units::degree_t> pivot_one_{*this, 
    "pivot_one", ports::arm_::kPivotOne_CANID};
  frc846::control::SparkRevController<units::degree_t> pivot_two_{*this, 
    "pivot_two", ports::arm_::kPivotTwo_CANID};
  frc846::control::SparkRevController<units::degree_t> pivot_three_{*this, 
    "pivot_three", ports::arm_::kPivotThree_CANID};
  frc846::control::SparkRevController<units::degree_t> pivot_four_{*this, 
    "pivot_four", ports::arm_::kPivotFour_CANID};

  // frc846::control::RevParallelController<units::degree_t> pivot_tandem_{*this, "pivot", {
  //   ports::arm_::kPivotOne_CANID,
  //   ports::arm_::kPivotTwo_CANID,
  //   ports::arm_::kPivotThree_CANID,
  //   ports::arm_::kPivotFour_CANID
  // }};
  
  frc846::control::SparkRevController<units::degree_t> wrist_esc_{*this, 
    "wrist_esc", ports::arm_::kWrist_CANID};

  frc846::control::SparkRevController<units::inch_t> tele_one_{*this, 
    "tele_one", ports::arm_::kTele1_CANID};
  frc846::control::SparkRevController<units::inch_t> tele_two_{*this, 
    "tele_two", ports::arm_::kTele2_CANID};


  // frc846::control::RevParallelController<units::inch_t> tele_tandem_{*this, "telescope", {
  //   ports::arm_::kTele1_CANID,
  //   ports::arm_::kTele2_CANID,
  // }};

  ArmReadings GetNewReadings() override;

  void PositionTelescope(ArmTarget target);
  void PositionPivot(ArmTarget target);
  void PositionWrist(ArmTarget target);

  void DirectWrite(ArmTarget target) override;
};

#endif