#ifndef y2024_SUBSYSTEMS_INTAKE_H_
#define y2024_SUBSYSTEMS_INTAKE_H_

#include "frc/AnalogInput.h"
#include "frc/AnalogTrigger.h"
#include "frc/filter/SlewRateLimiter.h"
#include "frc846/control/control.h"
#include "frc846/control/controlgains.h"
#include "frc846/loggable.h"
#include "frc846/subsystem.h"
#include "frc846/util/grapher.h"
#include "frc846/util/pref.h"
#include "ports.h"
#include "units/angular_velocity.h"

struct IntakeReadings {};

enum IntakeState { kIntake, kPull, kFeed, kRelease, kHold };

struct IntakeTarget {
  IntakeState target_state;
};

class IntakeSubsystem : public frc846::Subsystem<IntakeReadings, IntakeTarget> {
 public:
  IntakeSubsystem(bool init);

  IntakeTarget ZeroTarget() const override;
  IntakeTarget MakeTarget(IntakeState target_state);

  bool VerifyHardware() override;

  bool GetHasPiece() { return has_piece_; }

  frc846::Pref<units::turns_per_second_t> intake_speed_{*this, "intake_speed_",
                                                        70_tps};

  frc846::Pref<units::turns_per_second_t> intake_feed_speed_{
      *this, "intake_feed_speed_", 50_tps};

  frc846::Pref<double> release_speed_{*this, "release_speed", -0.3};

  frc846::Pref<double> retract_speed_{*this, "retract_speed", -0.2};

 private:
  bool has_piece_;

  double lastIntakeTarget;

  frc846::Loggable readings_named_{*this, "readings"};
  frc846::Grapher<bool> readings_has_piece_graph{readings_named_,
                                                 "readings_has_piece"};

  frc846::Grapher<double> readings_intake_speed_{readings_named_,
                                                 "readings_intake_speed"};

  frc846::Grapher<double> intake_error_{readings_named_, "intake_error"};
  frc846::Grapher<double> intake_current_draw_{readings_named_,
                                               "intake_current_draw"};

  frc846::Loggable target_named_{*this, "target"};
  frc846::Grapher<bool> target_is_intaking_graph{target_named_,
                                                 "target_is_intaking_graph"};

  frc846::Loggable intake_gains_lg = frc846::Loggable(*this, "intake_gains");
  frc846::control::ControlGainsHelper intake_esc_gains_{
      intake_gains_lg, {0, 0, 0, 0, 0}, 100_A, 0.5};

  frc846::control::SparkRevController<units::turn_t> intake_esc_{
      *this, "intake_esc_", ports::scorer_::kController_CANID};

  rev::SparkLimitSwitch note_detection;
  rev::SparkLimitSwitch note_detection_other;

  IntakeReadings GetNewReadings() override;

  void DirectWrite(IntakeTarget target) override;
};

#endif