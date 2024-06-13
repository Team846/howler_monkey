#ifndef y2024_SUBSYSTEMS_WRIST_H_
#define y2024_SUBSYSTEMS_WRIST_H_

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

struct WristReadings {
  units::degree_t wrist_position;
};

struct WristTarget {
  std::variant<units::degree_t, double> wrist_output;
};

class WristSubsystem : public frc846::Subsystem<WristReadings, WristTarget> {
 public:
  WristSubsystem(bool init);

  WristTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool GetHasZeroed() { return hasZeroed; }

  bool WithinTolerance(units::degree_t pos) {
    return (units::math::abs(pos - readings().wrist_position) <
            wrist_tolerance_.value());
  }

  void Coast() {
    wrist_esc_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  }

  void Brake() {
    wrist_esc_.esc_.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

  void ZeroSubsystem() {
    hasZeroed = true;
    wrist_esc_.ZeroEncoder();
    SetTarget(ZeroTarget());
  }

  frc846::Pref<units::degree_t> wrist_tolerance_{*this, "wrist_tolerance",
                                                 1.2_deg};

  frc846::Pref<units::degree_t> wrist_home_offset_{*this, "home_offset_wrist",
                                                   30_deg};
  frc846::Pref<units::degree_t> wrist_cg_offset_{*this, "cg_offset_wrist",
                                                 60_deg};

  frc846::Pref<units::degrees_per_second_t> max_adjustment_rate_{
      *this, "max_adjustment_rate", units::degrees_per_second_t(80.0)};

 private:
  bool hasZeroed = false;

  frc846::Loggable gains_{*this, "gains"};
  frc846::Pref<double> k_{gains_, "k", 0.0};
  frc846::Pref<double> p_{gains_, "p", 0.0};
  frc846::Pref<double> d_{gains_, "d", 0.0};

  frc846::Loggable readings_named_{*this, "readings"};

  frc846::Grapher<units::degree_t> wrist_pos_graph{target_named_, "wrist_pos"};
  frc846::Grapher<units::degree_t> wrist_error_graph{target_named_,
                                                     "wrist_error"};

  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<double> target_wrist_duty_cycle_graph{target_named_,
                                                        "wrist_duty_cycle"};

  frc846::Grapher<units::degree_t> target_wrist_pos_graph{target_named_,
                                                          "wrist_pos"};

  frc846::Loggable wrist_gains_lg = frc846::Loggable(*this, "wrist_gains");
  frc846::control::ControlGainsHelper wrist_esc_gains_{
      wrist_gains_lg, {0, 0, 0, 0, 0}, 200_A, 0.5};

  frc846::control::SparkRevController<units::degree_t> wrist_esc_{
      *this, "wrist_esc", ports::positioning_::kWrist_CANID};

  WristReadings GetNewReadings() override;

  void PositionWrist(WristTarget target);

  void DirectWrite(WristTarget target) override;
};

#endif