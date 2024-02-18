#ifndef y2024_SUBSYSTEMS_WRIST_H_
#define y2024_SUBSYSTEMS_WRIST_H_

#include "frc846/util/grapher.h"
#include "frc846/loggable.h"
#include "frc846/util/pref.h"
#include "frc846/subsystem.h"
#include "ports.h"
#include "frc846/control/control.h"
#include "frc846/control/controlgains.h"
#include "units/length.h"


struct WristReadings {
  units::degree_t wrist_position;
};

struct WristTarget {
  std::variant<units::degree_t, double> wrist_output;
};


class WristSubsystem
    : public frc846::Subsystem<WristReadings, WristTarget> {
 public:
  WristSubsystem(bool init);

  WristTarget ZeroTarget() const override;

  WristTarget MakeTarget(std::variant<units::degree_t, double> wrist_out);

  bool VerifyHardware() override;

  bool GetHasZeroed() { return hasZeroed; }

  void ZeroSubsystem() { 
    hasZeroed = true;
    wrist_esc_.ZeroEncoder();
    SetTarget(ZeroTarget());
  }

  frc846::Pref<units::degree_t> intake_setpoint_wrist_{*this, "intake_setpoint_wrist", 0_deg};

  frc846::Pref<units::degree_t> stow_setpoint_wrist_{*this, "stow_setpoint_wrist", 0_deg};

 private:
  bool hasZeroed = false;

  frc846::Loggable readings_named_{*this, "readings"};

  frc846::Grapher<units::degree_t> wrist_pos_graph{target_named_, "wrist_pos"};

  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<double> target_wrist_duty_cycle_graph{target_named_, "wrist_duty_cycle"};

  frc846::Grapher<units::degree_t> target_wrist_pos_graph{target_named_, "wrist_pos"};


  frc846::Loggable wrist_gains_lg = frc846::Loggable(*this, "wrist_gains");
  frc846::control::ControlGainsHelper wrist_esc_gains_{wrist_gains_lg,
    {0, 0, 0, 0, 0}, 
        200_A, 0.5};
  
  frc846::control::SparkRevController<units::degree_t> wrist_esc_{*this, 
    "wrist_esc", ports::positioning_::kWrist_CANID};

  WristReadings GetNewReadings() override;

  void PositionWrist(WristTarget target);

  void DirectWrite(WristTarget target) override;
};

#endif