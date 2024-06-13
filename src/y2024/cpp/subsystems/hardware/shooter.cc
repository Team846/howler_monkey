#include "subsystems/hardware/shooter.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

ShooterSubsystem::ShooterSubsystem(bool init)
    : frc846::Subsystem<ShooterReadings, ShooterTarget>{"shooter", init} {
  if (init) {
    shooter_esc_one_.DisableStatusFrames(
        {rev::CANSparkBase::PeriodicFrame::kStatus0,
         rev::CANSparkBase::PeriodicFrame::kStatus4,
         rev::CANSparkBase::PeriodicFrame::kStatus2,
         rev::CANSparkBase::PeriodicFrame::kStatus3,
         rev::CANSparkBase::PeriodicFrame::kStatus1,
         rev::CANSparkBase::PeriodicFrame::kStatus5});
    shooter_esc_two_.DisableStatusFrames(
        {rev::CANSparkBase::PeriodicFrame::kStatus0,
         rev::CANSparkBase::PeriodicFrame::kStatus4,
         rev::CANSparkBase::PeriodicFrame::kStatus2,
         rev::CANSparkBase::PeriodicFrame::kStatus3,
         rev::CANSparkBase::PeriodicFrame::kStatus1,
         rev::CANSparkBase::PeriodicFrame::kStatus5});

    shooter_esc_one_.Setup(&shooter_esc_gains_, true, frc846::control::kCoast);
    shooter_esc_one_.SetupConverter(1_tr);

    shooter_esc_two_.Setup(&shooter_esc_gains_, false, frc846::control::kCoast);
    shooter_esc_two_.SetupConverter(1_tr);
  }
}

ShooterTarget ShooterSubsystem::ZeroTarget() const {
  ShooterTarget target;
  target.target_state = kIdle;
  return target;
}

bool ShooterSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;
    FRC846_VERIFY(shooter_esc_one_.VerifyConnected(), ok,
                  "shooter one not connected");
    FRC846_VERIFY(shooter_esc_two_.VerifyConnected(), ok,
                  "shooter two not connected");
    return ok;
  }
  return true;
}

ShooterReadings ShooterSubsystem::GetNewReadings() {
  ShooterReadings readings;

  double shooter_one_velocity = shooter_esc_one_.GetVelocity().to<double>();
  double shooter_two_velocity = shooter_esc_two_.GetVelocity().to<double>();

  double leftError =
      (shooter_speed_.value().to<double>() * (1 + spin_.value()) -
       shooter_one_velocity) /
      (shooter_speed_.value().to<double>() * (1 + spin_.value()));
  double rightError =
      (shooter_speed_.value().to<double>() * (1 - spin_.value()) -
       shooter_two_velocity) /
      (shooter_speed_.value().to<double>() * (1 - spin_.value()));

  readings_shooting_speed_left_graph.Graph(shooter_one_velocity);
  readings_shooting_speed_right_graph.Graph(shooter_two_velocity);
  shooter_left_error_graph_.Graph(leftError);
  shooter_right_error_graph_.Graph(rightError);

  readings.error_percent = (leftError + rightError) / 2.0;

  return readings;
}

void ShooterSubsystem::DirectWrite(ShooterTarget target) {
  if (target.target_state == kRun) {
    shooter_esc_one_.Write(frc846::control::ControlMode::Velocity,
                           shooter_speed_.value() * (1 + spin_.value()));
    shooter_esc_two_.Write(frc846::control::ControlMode::Velocity,
                           shooter_speed_.value() * (1 - spin_.value()));
  } else {
    shooter_esc_one_.Write(frc846::control::ControlMode::Percent, 0.0);
    shooter_esc_two_.Write(frc846::control::ControlMode::Percent, 0.0);
  }
}