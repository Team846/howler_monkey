#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "frc846/base/loggable.h"
#include "frc846/control/control.h"
#include "frc846/control/motion.h"
#include "frc846/ntinf/grapher.h"
#include "frc846/ntinf/pref.h"
#include "frc846/robot/GenericSubsystem.h"
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

class PivotSubsystem
    : public frc846::robot::GenericSubsystem<PivotReadings, PivotTarget> {
 public:
  PivotSubsystem(bool init);

  void Setup() override;

  PivotTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool GetHasZeroed() { return hasZeroed; }

  bool WithinTolerance(units::degree_t pos) {
    return (units::math::abs(pos - GetReadings().pivot_position) <
            pivot_tolerance_.value());
  }

  void Coast() {
    if (auto esc = pivot_one_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    if (auto esc = pivot_two_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    if (auto esc = pivot_three_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    if (auto esc = pivot_four_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  }

  void Brake() {
    if (auto esc = pivot_one_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    if (auto esc = pivot_two_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    if (auto esc = pivot_three_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    if (auto esc = pivot_four_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

  void ZeroSubsystem() {
    hasZeroed = true;
    pivot_one_.ZeroEncoder(pivot_home_offset_.value());
    pivot_two_.ZeroEncoder(pivot_home_offset_.value());
    pivot_three_.ZeroEncoder(pivot_home_offset_.value());
    pivot_four_.ZeroEncoder(pivot_home_offset_.value());
    SetTarget(ZeroTarget());
  }

  bool WithinLimits(units::degree_t pos) {
    return hard_limits_.WithinLimits(pos);
  }

  units::degree_t CapWithinLimits(units::degree_t pos) {
    return hard_limits_.CapWithinLimits(pos);
  }

  frc846::ntinf::Pref<units::degree_t> pivot_tolerance_{
      *this, "pivot_tolerance", 0.5_deg};

  frc846::ntinf::Pref<units::degree_t> pivot_home_offset_{
      *this, "pivot_home_offset", -17_deg};

  frc846::ntinf::Pref<units::degrees_per_second_t> max_adjustment_rate_{
      *this, "max_adjustment_rate", units::degrees_per_second_t(60.0)};

 private:
  bool hasZeroed = false;

  frc846::base::Loggable readings_named_{*this, "readings"};

  frc846::ntinf::Grapher<units::degree_t> pivot_pos_graph{readings_named_,
                                                          "pivot_pos"};
  frc846::ntinf::Grapher<units::degree_t> pivot_error_graph{readings_named_,
                                                            "pivot_error"};

  frc846::base::Loggable target_named_{*this, "target"};

  frc846::ntinf::Grapher<double> target_pivot_duty_cycle_graph{
      target_named_, "pivot_duty_cycle"};

  frc846::ntinf::Grapher<units::degree_t> target_pivot_pos_graph{target_named_,
                                                                 "pivot_pos"};

  frc846::control::ConfigHelper config_helper_{
      *this,
      {false,
       (360.0) / (68.0 / 7.0 * 50.0 / 18.0 * 64.0 / 10.0),
       frc846::control::MotorIdleMode::kDefaultBrake,
       {60_A}},
      {0.047, 0.00, 0.00}};

  frc846::control::HardLimitsConfigHelper<units::degree_t> hard_limits_{
      *this, {110_deg, 0_deg, true, 0.7, -0.5}};

  frc846::control::REVSparkController<units::degree_t> pivot_one_{
      *this, ports::positioning_::kPivotOne_CANID, config_helper_,
      hard_limits_};
  frc846::control::REVSparkController<units::degree_t> pivot_two_{
      *this, ports::positioning_::kPivotTwo_CANID, config_helper_,
      hard_limits_};
  frc846::control::REVSparkController<units::degree_t> pivot_three_{
      *this, ports::positioning_::kPivotThree_CANID, config_helper_,
      hard_limits_};
  frc846::control::REVSparkController<units::degree_t> pivot_four_{
      *this, ports::positioning_::kPivotFour_CANID, config_helper_,
      hard_limits_};

  PivotReadings ReadFromHardware() override;

  void WriteToHardware(PivotTarget target) override;

  frc846::base::Loggable dyFPID_loggable{*this, "DynamicFPID"};

  frc846::motion::BrakingPositionDyFPID<units::degree_t> dyFPID{
      dyFPID_loggable,
      [this](units::degree_t pos) -> double {
        return std::abs(
            units::math::cos(GetReadings().pivot_position).to<double>());
      },
      {30_A, frc846::control::DefaultSpecifications::stall_current_neo, 0.3}};
};
