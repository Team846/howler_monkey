#ifndef y2024_SUBSYSTEMS_SHINTAKE_H_
#define y2024_SUBSYSTEMS_SHINTAKE_H_

#include "frc846/grapher.h"
#include "frc846/loggable.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "ports.h"
#include "frc846/control/control.h"
#include "frc846/control/newgains.h"
#include "units/angular_velocity.h"
#include "frc/AnalogInput.h"
#include "frc/AnalogTrigger.h"


struct ShintakeReadings {
  
};

struct ShintakeTarget {
  bool run_intake;
  bool run_shooter;
  units::turns_per_second_t shooter_speed;
};


class ShintakeSubsystem
    : public frc846::Subsystem<ShintakeReadings, ShintakeTarget> {
 public:
  ShintakeSubsystem(bool init);

  ShintakeTarget ZeroTarget() const override;
  ShintakeTarget MakeTarget(bool intake, bool shoot, units::turns_per_second_t shoot_spd);

  bool VerifyHardware() override;

  bool GetHasPiece() { return has_piece_; }

  frc846::Pref<double> intake_speed_{*this, "intake_speed_", 0.3};
  frc846::Pref<double> intake_feed_speed_{*this, "intake_feed_speed_", 0.1};
  frc846::Pref<units::turns_per_second_t> shooter_speed_{*this, "shooter_speed_", 50_tps};

 private:
  bool has_piece_;

  frc846::Loggable readings_named_{*this, "readings"};
  frc846::Grapher<bool> readings_has_piece_graph{readings_named_, "readings_has_piece"};

  frc846::Loggable target_named_{*this, "target"};
  frc846::Grapher<bool> target_is_intaking_graph{target_named_, "target_is_intaking_graph"};
  frc846::Grapher<bool> target_is_shooting_graph{target_named_, "target_is_shooting_graph"};
  frc846::Grapher<double> target_shooting_speed_graph{target_named_, "target_shooting_speed_graph"};


  frc846::Loggable intake_gains_lg = frc846::Loggable(*this, "intake_gains");
  frc846::control::ControlGainsHelper intake_esc_gains_{intake_gains_lg,
    {0, 0, 0, 0, 0}, 
        100_A, 0.5};

  frc846::Loggable shooter_gains_lg = frc846::Loggable(*this, "shooter_gains");
  frc846::control::ControlGainsHelper shooter_esc_gains_{shooter_gains_lg,
    {0, 0, 0, 0, 0}, 
        200_A, 0.5};

  frc846::control::SparkRevController<units::turn_t> intake_shooter_esc_{*this, 
    "intake_shooter_esc_", ports::shintake_::kController_CANID};


  frc::AnalogTrigger note_detector_{0};


  ShintakeReadings GetNewReadings() override;

  void DirectWrite(ShintakeTarget target) override;
};

#endif