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
#include "commands/follow_trajectory_command.h"
#include "commands/teleop/bracer_command.h"
#include "commands/teleop/drive_command.h"
#include "commands/teleop/idle_intake_command.h"
#include "commands/teleop/idle_shooter_command.h"
#include "commands/teleop/leds_command.h"
#include "commands/teleop/operator_control.h"
#include "commands/teleop/stow_command.h"
#include "control_triggers.h"
#include "frc846/other/sendable_callback.h"

FunkyRobot::FunkyRobot() : GenericRobot{&container_} {}

void FunkyRobot::OnInitialize() {
  // Add dashboard buttons
  frc::SmartDashboard::PutData(
      "zero_modules", new frc846::other::SendableCallback(
                          [this] { container_.drivetrain_.ZeroModules(); }));
  frc::SmartDashboard::PutData("zero_bearing",
                               new frc846::other::SendableCallback([this] {
                                 container_.drivetrain_.SetBearing(0_deg);
                               }));

  frc::SmartDashboard::PutData(
      "zero_odometry", new frc846::other::SendableCallback(
                           [this] { container_.drivetrain_.ZeroOdometry(); }));

  frc::SmartDashboard::PutData("zero_subsystems",
                               new frc846::other::SendableCallback([this] {
                                 container_.super_structure_.ZeroSubsystem();
                               }));

  frc::SmartDashboard::PutData("coast_subsystems",
                               new frc846::other::SendableCallback([this] {
                                 container_.pivot_.Coast();
                                 container_.telescope_.Coast();
                                 container_.wrist_.Coast();
                               }));

  frc::SmartDashboard::PutData("brake_subsystems",
                               new frc846::other::SendableCallback([this] {
                                 container_.pivot_.Brake();
                                 container_.telescope_.Brake();
                                 container_.wrist_.Brake();
                               }));

  AddAutos(drive_auto_.get(),
           {five_piece_auto_red.get(), five_piece_auto_blue.get(),
            one_piece_auto_0.get(), one_piece_auto_1.get(),
            one_piece_auto_2.get(), one_piece_auto_3.get()});
}

void FunkyRobot::InitTeleop() {
  container_.pivot_.Brake();
  container_.telescope_.Brake();
  container_.wrist_.Brake();

  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
  container_.control_input_.SetDefaultCommand(
      OperatorControlCommand{container_});
  container_.super_structure_.SetDefaultCommand(StowCommand{container_});
  container_.intake_.SetDefaultCommand(IdleIntakeCommand{container_});
  container_.shooter_.SetDefaultCommand(IdleShooterCommand{container_});
  container_.bracer_.SetDefaultCommand(BracerCommand{container_});

  frc2::Trigger drivetrain_zero_bearing_trigger{
      [&] { return container_.control_input_.GetReadings().zero_bearing; }};

  drivetrain_zero_bearing_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.drivetrain_.SetBearing(
            frc846::util::ShareTables::GetBoolean("is_red_side") ? 0_deg
                                                                 : 180_deg);
      }).ToPtr());

  frc2::Trigger on_piece_trigger{[&] {
    return frc846::util::ShareTables::GetBoolean("scorer_has_piece");
  }};

  on_piece_trigger.OnTrue(
      frc2::InstantCommand(
          [this] { container_.control_input_.SetTarget({true, false}); })
          .WithTimeout(1_s)
          .AndThen(frc2::WaitCommand(1_s).ToPtr())
          .AndThen(frc2::InstantCommand([this] {
                     container_.control_input_.SetTarget({false, false});
                   }).ToPtr()));

  ControlTriggerInitializer::InitTeleopTriggers(container_);
}

void FunkyRobot::InitTest() {}
