#include "FunkyRobot.h"

#include <cameraserver/CameraServer.h>
#include <frc/DSControlWord.h>
#include <frc/Filesystem.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "commands/basic/amp_command.h"
#include "commands/basic/deploy_intake_command.h"
#include "commands/basic/shoot_command.h"
#include "commands/basic/spin_up_command.h"
#include "commands/complex/stow_zero_action.h"
#include "commands/teleop/bracer_command.h"
#include "commands/teleop/drive_command.h"
#include "commands/teleop/idle_intake_command.h"
#include "commands/teleop/idle_shooter_command.h"
#include "commands/teleop/leds_command.h"
#include "commands/teleop/operator_control.h"
#include "commands/teleop/stow_command.h"
#include "control_triggers.h"
#include "field.h"
#include "frc846/ntinf/ntaction.h"
#include "frc846/swerve/follow_trajectory_command.h"
#include "rsighandler.h"

FunkyRobot::FunkyRobot() : GenericRobot{&container_} {}

void FunkyRobot::OnInitialize() {
  container_.leds_.SetDefaultCommand(LEDsCommand{container_});

  Field::Setup();

  for (auto x : Field::getAllAutoData()) {
    Log("Adding Auto: {}", x.name + "_red");
    AddAuto(x.name + "_red", new GenericAuto{container_, x, false});
    Log("Adding Auto: {}", x.name + "_blue");
    AddAuto(x.name + "_blue", new GenericAuto{container_, x, true});
  }

  // Add dashboard buttons
  frc::SmartDashboard::PutData(
      "zero_modules", new frc846::ntinf::NTAction(
                          [this] { container_.drivetrain_.ZeroModules(); }));
  frc::SmartDashboard::PutData("zero_bearing",
                               new frc846::ntinf::NTAction([this] {
                                 container_.drivetrain_.SetBearing(0_deg);
                               }));

  frc::SmartDashboard::PutData(
      "zero_odometry", new frc846::ntinf::NTAction(
                           [this] { container_.drivetrain_.ZeroOdometry(); }));

  frc::SmartDashboard::PutData("zero_subsystems",
                               new frc846::ntinf::NTAction([this] {
                                 container_.super_structure_.ZeroSubsystem();
                               }));

  frc::SmartDashboard::PutData("coast_subsystems",
                               new frc846::ntinf::NTAction([this] {
                                 container_.pivot_.Coast();
                                 container_.telescope_.Coast();
                                 container_.wrist_.Coast();
                               }));

  frc::SmartDashboard::PutData("brake_subsystems",
                               new frc846::ntinf::NTAction([this] {
                                 container_.pivot_.Brake();
                                 container_.telescope_.Brake();
                                 container_.wrist_.Brake();
                               }));
}

void FunkyRobot::OnDisable() {
  container_.leds_.SetDefaultCommand(LEDsCommand{container_});

  container_.pivot_.Brake();
  container_.wrist_.Brake();
  container_.telescope_.Brake();
}

void FunkyRobot::InitTeleop() {
  container_.pivot_.Brake();
  container_.telescope_.Brake();
  container_.wrist_.Coast();

  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
  container_.control_input_.SetDefaultCommand(
      OperatorControlCommand{container_});
  container_.super_structure_.SetDefaultCommand(StowCommand{container_});
  container_.intake_.SetDefaultCommand(IdleIntakeCommand{container_});
  container_.shooter_.SetDefaultCommand(IdleShooterCommand{container_});
  container_.bracer_.SetDefaultCommand(BracerCommand{container_});
  container_.leds_.SetDefaultCommand(LEDsCommand{container_});

  ControlTriggerInitializer::InitTeleopTriggers(container_);
}

void FunkyRobot::OnPeriodic() {
  Graph("homing_switch", !homing_switch_.Get());
  Graph("coasting_switch", !coasting_switch_.Get());

  if (!homing_switch_.Get()) {
    container_.super_structure_.ZeroSubsystem();
    frc846::util::ShareTables::SetBoolean("zero sequence", true);
    Log("Zeroing subsystems...");
  }

  if (frc846::wpilib::CurrentFPGATime() > stop_coast_time_) {
    container_.pivot_.Brake();
    container_.wrist_.Brake();
    container_.telescope_.Brake();

    stop_coast_time_ =
        frc846::wpilib::CurrentFPGATime() +
        coasting_time_
            .value();  // To prevent Brake from being called each periodic

  } else if (!coasting_switch_.Get()) {
    container_.pivot_.Coast();
    container_.wrist_.Coast();
    container_.telescope_.Coast();

    stop_coast_time_ =
        frc846::wpilib::CurrentFPGATime() + coasting_time_.value();

    Log("Coasting subsystems...");
  }
}

void FunkyRobot::InitTest() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  // configureSignalHandlers();
  return frc::StartRobot<FunkyRobot>();
}
#endif