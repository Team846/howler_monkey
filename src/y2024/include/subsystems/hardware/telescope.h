#ifndef y2024_SUBSYSTEMS_TELESCOPE_H_
#define y2024_SUBSYSTEMS_TELESCOPE_H_

#include "frc846/control/control.h"
#include "frc846/control/controlgains.h"
#include "frc846/loggable.h"
#include "frc846/subsystem.h"
#include "frc846/util/grapher.h"
#include "frc846/util/pref.h"
#include "ports.h"
#include "units/length.h"
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

  TelescopeTarget MakeTarget(std::variant<units::inch_t, double> tele_out);

  bool VerifyHardware() override;

  bool GetHasZeroed() { return hasZeroed; }

  void Coast() {
    telescope_esc_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  }

  void Brake() {
    telescope_esc_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

  void ZeroSubsystem() {
    hasZeroed = true;
    telescope_esc_.ZeroEncoder();
    SetTarget(ZeroTarget());
  }

  frc846::Pref<units::feet_per_second_t> max_adjustment_rate_{
      *this, "max_adjustment_rate", 0.5_fps};

 private:
  bool hasZeroed = false;

  frc846::Loggable readings_named_{*this, "readings"};

  frc846::Grapher<units::inch_t> tele_pos_graph{readings_named_, "tele_pos"};
  frc846::Grapher<units::inch_t> tele_error_graph{readings_named_,
                                                  "tele_error"};

  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<double> target_tele_duty_cycle_graph{target_named_,
                                                       "tele_duty_cycle"};

  frc846::Grapher<units::inch_t> target_tele_pos_graph{target_named_,
                                                       "tele_pos"};

  frc846::Loggable tele_gains_lg = frc846::Loggable(*this, "tele_gains");
  frc846::control::ControlGainsHelper tele_esc_gains_{
      tele_gains_lg, {0, 0, 0, 0, 0}, 50_A, 0.5};

  frc846::control::SparkRevController<units::inch_t> telescope_esc_{
      *this, "telescope_esc", ports::positioning_::kTele_CANID};

  TelescopeReadings GetNewReadings() override;

  void PositionTelescope(TelescopeTarget target);

  void DirectWrite(TelescopeTarget target) override;
};

#endif