#pragma once

#include "frc846/control/control.h"
#include "frc846/loggable.h"
#include "frc846/subsystem.h"
#include "frc846/util/grapher.h"
#include "frc846/util/pref.h"
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
    : public frc846::Subsystem<TelescopeReadings, TelescopeTarget> {
 public:
  TelescopeSubsystem(bool init);

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
    return (units::math::abs(pos - readings().extension) <
            telescope_tolerance_.value());
  }

  frc846::Pref<units::feet_per_second_t> max_adjustment_rate_{
      *this, "max_adjustment_rate", 0.5_fps};

 private:
  bool hasZeroed = false;

  frc846::Pref<units::inch_t> telescope_tolerance_{*this, "telescope_tolerance",
                                                   0.25_in};

  frc846::Loggable readings_named_{*this, "readings"};

  frc846::Grapher<units::inch_t> tele_pos_graph{readings_named_, "tele_pos"};
  frc846::Grapher<units::inch_t> tele_error_graph{readings_named_,
                                                  "tele_error"};

  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<double> target_tele_duty_cycle_graph{target_named_,
                                                       "tele_duty_cycle"};

  frc846::Grapher<units::inch_t> target_tele_pos_graph{target_named_,
                                                       "tele_pos"};

  frc846::control::ConfigHelper config_helper_{
      *this,
      {false, 0.25, frc846::control::MotorIdleMode::kDefaultBrake, {40_A}},
      {0.3, 0.00, 0.00}};

  frc846::control::HardLimitsConfigHelper<units::inch_t> hard_limits_{
      *this, {6_in, 0_in, true, 1.0, -0.7}};

  frc846::control::REVSparkController<units::inch_t> telescope_esc_{
      *this, ports::positioning_::kTele_CANID, config_helper_, hard_limits_};

  TelescopeReadings GetNewReadings() override;

  void DirectWrite(TelescopeTarget target) override;
};
