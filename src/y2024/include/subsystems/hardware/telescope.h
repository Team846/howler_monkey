#pragma once

#include "frc846/base/loggable.h"
#include "frc846/control/control.h"
#include "frc846/ntinf/grapher.h"
#include "frc846/ntinf/pref.h"
#include "frc846/robot/GenericSubsystem.h"
#include "ports.h"
#include "units/length.h"
#include "units/math.h"
#include "units/velocity.h"

struct TelescopeReadings {
  units::inch_t extension;
};

struct TelescopeTarget {
  std::variant<units::inch_t, double> extension;
};

class TelescopeSubsystem
    : public frc846::robot::GenericSubsystem<TelescopeReadings,
                                             TelescopeTarget> {
 public:
  TelescopeSubsystem(bool init);

  void Setup() override;

  TelescopeTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool GetHasZeroed() { return hasZeroed; }

  void Coast() {
    if (auto esc = telescope_esc_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  }

  void Brake() {
    if (auto esc = telescope_esc_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

  void ZeroSubsystem() {
    hasZeroed = true;
    telescope_esc_.ZeroEncoder(0.0_in);
    SetTarget(ZeroTarget());
  }

  bool WithinTolerance(units::inch_t pos) {
    return (units::math::abs(pos - GetReadings().extension) <
            telescope_tolerance_.value());
  }

  bool WithinLimits(units::inch_t pos) {
    return hard_limits_.WithinLimits(pos);
  }

  units::inch_t CapWithinLimits(units::inch_t pos) {
    return hard_limits_.CapWithinLimits(pos);
  }

  frc846::ntinf::Pref<units::feet_per_second_t> max_adjustment_rate_{
      *this, "max_adjustment_rate", 0.5_fps};

 private:
  bool hasZeroed = false;

  frc846::ntinf::Pref<units::inch_t> telescope_tolerance_{
      *this, "telescope_tolerance", 0.25_in};

  frc846::base::Loggable readings_named_{*this, "readings"};

  frc846::ntinf::Grapher<units::inch_t> tele_pos_graph{readings_named_,
                                                       "tele_pos"};
  frc846::ntinf::Grapher<units::inch_t> tele_error_graph{readings_named_,
                                                         "tele_error"};

  frc846::base::Loggable target_named_{*this, "target"};

  frc846::ntinf::Grapher<double> target_tele_duty_cycle_graph{
      target_named_, "tele_duty_cycle"};

  frc846::ntinf::Grapher<units::inch_t> target_tele_pos_graph{target_named_,
                                                              "tele_pos"};

  frc846::control::ConfigHelper config_helper_{
      *this,
      {false, 0.25, frc846::control::MotorIdleMode::kDefaultBrake, {40_A}},
      {0.3, 0.00, 0.00}};

  frc846::control::HardLimitsConfigHelper<units::inch_t> hard_limits_{
      *this, {6_in, 0_in, true, 1.0, -0.7}};

  frc846::control::REVSparkController<units::inch_t> telescope_esc_{
      *this, ports::positioning_::kTele_CANID, config_helper_, hard_limits_};

  TelescopeReadings ReadFromHardware() override;

  void WriteToHardware(TelescopeTarget target) override;
};
