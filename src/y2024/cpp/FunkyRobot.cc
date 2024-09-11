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

  frc2::Trigger on_coast_trigger{[&] { return coasting_switch_.Get(); }};

  on_coast_trigger.OnTrue(frc2::InstantCommand([&] {
                            container_.pivot_.Coast();
                            container_.telescope_.Coast();
                          })
                              .WithTimeout(1_s)
                              .AndThen(frc2::WaitCommand(7_s).ToPtr())
                              .AndThen(frc2::InstantCommand([&] {
                                         container_.pivot_.Brake();
                                         container_.telescope_.Brake();
                                       }).ToPtr()));

  frc2::Trigger homing_trigger{[&] { return homing_switch_.Get(); }};

  homing_trigger.OnTrue(frc2::InstantCommand([&] {
                          container_.wrist_.ZeroSubsystem();
                          container_.pivot_.ZeroSubsystem();
                          container_.telescope_.ZeroSubsystem();
                        }).ToPtr());

  AddAutos(five_piece_auto_red.get(),
           {five_piece_auto_blue.get(), one_piece_auto_0.get(),
            one_piece_auto_1.get(), one_piece_auto_2.get(),
            one_piece_auto_3.get(), drive_auto_.get()});
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

  ControlTriggerInitializer::InitTeleopTriggers(container_);
}

void FunkyRobot::InitTest() {}
