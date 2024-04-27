#ifndef y2024_SUBSYSTEMS_TELESCOPE_H_
#define y2024_SUBSYSTEMS_TELESCOPE_H_

#include "frc846/util/grapher.h"
#include "frc846/loggable.h"
#include "frc846/util/pref.h"
#include "frc846/subsystem.h"
#include "ports.h"
#include "frc846/control/control.h"
#include "frc846/control/controlgains.h"
#include "units/length.h"


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
    tele_one_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  }

  void Brake() {
    tele_one_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

  void ZeroSubsystem() { 
    hasZeroed = true;
    tele_one_.ZeroEncoder();
    // tele_two_.ZeroEncoder();
    SetTarget(ZeroTarget());
  }

  frc846::Pref<units::inch_t> intake_setpoint_tele_{*this, "intake_setpoint_tele_", 0_in};

  frc846::Pref<units::inch_t> stow_setpoint_tele_{*this, "stow_setpoint_tele_", 0_in};

  frc846::Pref<units::inch_t> telescope_position_tolerance_{*this, "telescope_position_tolerance", 0.1_in};

  frc846::Pref<double> telescope_adj_inc_{*this, "telescope_adj_inc_", 0.06};

  // frc846::Pref<double> telescope_home_speed_{*this, "telescope_home_speed", -0.1};

 private:
  bool hasZeroed = false;

  frc846::Loggable readings_named_{*this, "readings"};

  frc846::Grapher<units::inch_t> tele_pos_graph{target_named_, "tele_pos"};


  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<double> target_tele_duty_cycle_graph{target_named_, "tele_duty_cycle"};

  frc846::Grapher<units::inch_t> target_tele_pos_graph{target_named_, "tele_pos"};


  frc846::Loggable tele_gains_lg = frc846::Loggable(*this, "tele_gains");
  frc846::control::ControlGainsHelper tele_esc_gains_{tele_gains_lg,
    {0, 0, 0, 0, 0}, 
        50_A, 0.5};


  frc846::control::SparkRevController<units::inch_t> tele_one_{*this, 
    "tele_one", 24}; //ports::positioning_::kTele1_CANID};
  // frc846::control::SparkRevController<units::inch_t> tele_two_{*this, 
  //   "tele_two", ports::positioning_::kTele2_CANID};


  TelescopeReadings GetNewReadings() override;

  void PositionTelescope(TelescopeTarget target);

  void DirectWrite(TelescopeTarget target) override;
};

#endif