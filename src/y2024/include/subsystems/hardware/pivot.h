#ifndef y2024_SUBSYSTEMS_ARM_H_
#define y2024_SUBSYSTEMS_ARM_H_

#include "frc/filter/SlewRateLimiter.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "frc846/control/control.h"
#include "frc846/control/controlgains.h"
#include "frc846/loggable.h"
#include "frc846/subsystem.h"
#include "frc846/util/grapher.h"
#include "frc846/util/pref.h"
#include "ports.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/math.h"

struct PivotReadings {
  units::degree_t pivot_position;
};

struct PivotTarget {
  std::variant<units::degree_t, double> pivot_output;
};

class PivotSubsystem : public frc846::Subsystem<PivotReadings, PivotTarget> {
 public:
  PivotSubsystem(bool init);

  PivotTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool GetHasZeroed() { return hasZeroed; }

  bool WithinTolerance(units::degree_t pos) {
    return (units::math::abs(pos - readings().pivot_position) <
            pivot_tolerance_.value());
  }

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

  frc846::Pref<units::degree_t> pivot_tolerance_{*this, "pivot_tolerance",
                                                 0.5_deg};

  frc846::Pref<units::degree_t> pivot_home_offset_{*this, "pivot_home_offset",
                                                   17_deg};

  frc846::Pref<units::degrees_per_second_t> max_adjustment_rate_{
      *this, "max_adjustment_rate", units::degrees_per_second_t(60.0)};

 private:
  bool hasZeroed = false;

  frc846::Loggable readings_named_{*this, "readings"};

  frc846::Grapher<units::degree_t> pivot_pos_graph{readings_named_,
                                                   "pivot_pos"};
  frc846::Grapher<units::degree_t> pivot_error_graph{readings_named_,
                                                     "pivot_error"};

  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<double> target_pivot_duty_cycle_graph{target_named_,
                                                        "pivot_duty_cycle"};

  frc846::Grapher<units::degree_t> target_pivot_pos_graph{target_named_,
                                                          "pivot_pos"};

  frc846::Loggable pivot_gains_lg = frc846::Loggable(*this, "pivot_gains");
  frc846::control::ControlGainsHelper pivot_esc_gains_{
      pivot_gains_lg, {0, 0, 0, 0, 0}, 100_A, 0.5};

  frc846::control::SparkFlexController<units::degree_t> pivot_one_{
      *this, "pivot_one", ports::positioning_::kPivotOne_CANID};
  frc846::control::SparkFlexController<units::degree_t> pivot_two_{
      *this, "pivot_two", ports::positioning_::kPivotTwo_CANID};
  frc846::control::SparkFlexController<units::degree_t> pivot_three_{
      *this, "pivot_three", ports::positioning_::kPivotThree_CANID};
  frc846::control::SparkFlexController<units::degree_t> pivot_four_{
      *this, "pivot_four", ports::positioning_::kPivotFour_CANID};

  PivotReadings GetNewReadings() override;

  void PositionTelescope(PivotTarget target);
  void PositionPivot(PivotTarget target);
  void PositionWrist(PivotTarget target);

  void DirectWrite(PivotTarget target) override;
};

#endif