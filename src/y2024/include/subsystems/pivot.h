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
#include "frc/trajectory/TrapezoidProfile.h"


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

  void Coast() {
    pivot_one_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    pivot_two_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    pivot_three_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    pivot_four_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  }

  void Brake() {
    pivot_one_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    pivot_two_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    pivot_three_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    pivot_four_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

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

  frc846::Pref<units::degree_t> ramp_rate_limit{
      *this, "ramp_rate_limit_", 40_deg};

  frc846::Loggable service_loggable{*this, "service_mode"};
  frc846::Pref<units::degree_t> service_forward_increment{service_loggable, "forward_increment", 0.0_deg};
  frc846::Pref<units::degree_t> service_backward_increment{service_loggable, "backward_increment", 0.0_deg};

  units::degree_t target_pivot_position=0_deg;

 private:
  frc::TrapezoidProfile<units::degree> ramp_rate_{
    frc::TrapezoidProfile<units::degree>::Constraints{ramp_rate_limit.value() / 1_s, 
      2.0 * ramp_rate_limit.value() / (1_s * 1_s)}};

  frc::TrapezoidProfile<units::degree>::State goal;
  frc::TrapezoidProfile<units::degree>::State progress;


  bool hasZeroed = false;

  frc846::Loggable readings_named_{*this, "readings"};

  frc846::Grapher<units::degree_t> pivot_pos_graph{target_named_, "pivot_pos"};


  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<double> target_pivot_duty_cycle_graph{target_named_, "pivot_duty_cycle"};

  frc846::Grapher<units::degree_t> target_pivot_pos_graph{target_named_, "pivot_pos"};
  frc846::Grapher<units::degree_t> intermediate_pivot_pos_graph{target_named_, "intermediate_pivot_pos"};


  frc846::Loggable pivot_gains_lg = frc846::Loggable(*this, "pivot_gains");
  frc846::control::ControlGainsHelper pivot_esc_gains_{pivot_gains_lg,
    {0, 0, 0, 0, 0}, 
        100_A, 0.5};

  frc846::control::SparkFlexController<units::degree_t> pivot_one_{*this, 
    "pivot_one", ports::positioning_::kPivotOne_CANID};
  frc846::control::SparkFlexController<units::degree_t> pivot_two_{*this, 
    "pivot_two", ports::positioning_::kPivotTwo_CANID};
  frc846::control::SparkFlexController<units::degree_t> pivot_three_{*this, 
    "pivot_three", ports::positioning_::kPivotThree_CANID};
  frc846::control::SparkFlexController<units::degree_t> pivot_four_{*this, 
    "pivot_four", ports::positioning_::kPivotFour_CANID};

  PivotReadings GetNewReadings() override;

  void PositionTelescope(PivotTarget target);
  void PositionPivot(PivotTarget target);
  void PositionWrist(PivotTarget target);

  void DirectWrite(PivotTarget target) override;
};

#endif