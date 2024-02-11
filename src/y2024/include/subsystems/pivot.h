#ifndef y2024_SUBSYSTEMS_ARM_H_
#define y2024_SUBSYSTEMS_ARM_H_

#include "frc846/util/grapher.h"
#include "frc846/loggable.h"
#include "frc846/util/pref.h"
#include "frc846/subsystem.h"
#include "ports.h"
#include "frc846/control/control.h"
#include "frc846/control/controlgains.h"
#include "units/length.h"


struct PivotReadings {
  units::degree_t pivot_position;
};

struct PivotTarget {
  std::variant<units::degree_t, double> pivot_output;
};


class PivotSubsystem
    : public frc846::Subsystem<PivotReadings, PivotTarget> {
 public:
  PivotSubsystem(bool init);

  PivotTarget ZeroTarget() const override;

  PivotTarget MakeTarget(std::variant<units::degree_t, double> pivot_out);

  bool VerifyHardware() override;

  bool GetHasZeroed() { return hasZeroed; }

  void ZeroSubsystem() { 
    hasZeroed = true;
    pivot_one_.ZeroEncoder();
    pivot_two_.ZeroEncoder();
    pivot_three_.ZeroEncoder();
    pivot_four_.ZeroEncoder();
    SetTarget(ZeroTarget());
  }

  frc846::Pref<units::degree_t> intake_setpoint_pivot{*this, "intake_setpoint_pivot", 0_deg};

  frc846::Pref<units::degree_t> stow_setpoint_pivot{*this, "stow_setpoint_pivot", 0_deg};

 private:
  bool hasZeroed = false;

  frc846::Loggable readings_named_{*this, "readings"};

  frc846::Grapher<units::degree_t> pivot_pos_graph{target_named_, "pivot_pos"};


  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<double> target_pivot_duty_cycle_graph{target_named_, "pivot_duty_cycle"};

  frc846::Grapher<units::degree_t> target_pivot_pos_graph{target_named_, "pivot_pos"};


  frc846::Loggable pivot_gains_lg = frc846::Loggable(*this, "pivot_gains");
  frc846::control::ControlGainsHelper pivot_esc_gains_{pivot_gains_lg,
    {0, 0, 0, 0, 0}, 
        100_A, 0.5};

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

  PivotReadings GetNewReadings() override;

  void PositionTelescope(PivotTarget target);
  void PositionPivot(PivotTarget target);
  void PositionWrist(PivotTarget target);

  void DirectWrite(PivotTarget target) override;
};

#endif