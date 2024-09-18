#include "FunkyRobot.h"

#include <cameraserver/CameraServer.h>
#include <frc/DSControlWord.h>
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
#include "commands/follow_trajectory_command.h"
#include "commands/teleop/bracer_command.h"
#include "commands/teleop/drive_command.h"
#include "commands/teleop/idle_intake_command.h"
#include "commands/teleop/idle_shooter_command.h"
#include "commands/teleop/leds_command.h"
#include "commands/teleop/operator_control.h"
#include "commands/teleop/stow_command.h"
#include "control_triggers.h"
#include "frc846/ntinf/ntaction.h"

FunkyRobot::FunkyRobot() : GenericRobot{&container_} {}

void FunkyRobot::OnInitialize() {
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

  AddAutos(five_piece_auto_red.get(),
           {five_piece_auto_blue.get(), one_piece_auto_0.get(),
            one_piece_auto_1.get(), one_piece_auto_2.get(),
            one_piece_auto_3.get(), drive_auto_.get()});
}

void FunkyRobot::OnDisable() {
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
  container_.super_structure_.SetDefaultCommand(
      StowZeroActionCommand{container_});
  container_.intake_.SetDefaultCommand(IdleIntakeCommand{container_});
  container_.shooter_.SetDefaultCommand(IdleShooterCommand{container_});
  container_.bracer_.SetDefaultCommand(BracerCommand{container_});

  ControlTriggerInitializer::InitTeleopTriggers(container_);
}

void FunkyRobot::OnPeriodic() {
  Graph("homing_switch", homing_switch_.Get());
  Graph("coasting_switch", coasting_switch_.Get());

  if (homing_switch_.Get()) {
    container_.super_structure_.ZeroSubsystem();
  }

  if (coast_counter_ >= 1) {
    coast_counter_--;
  } else if (coast_counter_ == 0) {
    container_.pivot_.Brake();
    container_.wrist_.Brake();
    container_.telescope_.Brake();
  } else if (coasting_switch_.Get()) {
    container_.pivot_.Coast();
    container_.wrist_.Coast();
    container_.telescope_.Coast();
    coast_counter_ = start_coast_counter_.value();
  }
}

void FunkyRobot::InitTest() {}
