#include "FunkyRobot.h"

#include <frc/DSControlWord.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "frc/DataLogManager.h"
#include "frc2/command/ParallelDeadlineGroup.h"
#include "frc2/command/WaitCommand.h"
#include "frc846/loggable.h"
#include "frc846/sendable_callback.h"
#include "frc846/wpilib/time.h"
#include "frc846/xbox.h"
#include "commands/drive_command.h"
#include "commands/follow_trajectory_command.h"

#include "subsystems/shintake.h"
#include "subsystems/pivot.h"
#include "subsystems/wrist.h"
#include "subsystems/telescope.h"
#include "commands/stow_command.h"
#include "commands/deploy_intake_command.h"

FunkyRobot::FunkyRobot() : frc846::Loggable{"funky_robot"} {
  next_loop_time_ = frc846::wpilib::CurrentFPGATime();

  int32_t status = 0;
  notifier_ = HAL_InitializeNotifier(&status);
  FRC_CheckErrorStatus(status, "{}", "InitializeNotifier");

  HAL_SetNotifierName(notifier_, "FunkyRobot", &status);
}

FunkyRobot::~FunkyRobot() {
  int32_t status = 0;
  HAL_StopNotifier(notifier_, &status);
  HAL_CleanNotifier(notifier_, &status);
}

void FunkyRobot::StartCompetition() {
  // Silence warnings related to missing joystick
  // (Doesn't do anything when connected to FMS)

  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // Disable live window
  frc::LiveWindow::DisableAllTelemetry();

  frc::DataLogManager::Start();

  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  // Add dashboard buttons
  frc::SmartDashboard::PutData(
      "zero_modules", new frc846::SendableCallback(
                          [this] { container_.drivetrain_.ZeroModules(); }));
  frc::SmartDashboard::PutData("zero_bearing",
                               new frc846::SendableCallback([this] {
                                 container_.drivetrain_.SetBearing(0_deg);
                               }));
  frc::SmartDashboard::PutData(
      "zero_odometry", new frc846::SendableCallback(
                           [this] { container_.drivetrain_.ZeroOdometry(); }));
  
  frc::SmartDashboard::PutData(
      "verify_hardware",
      new frc846::SendableCallback([this] { VerifyHardware(); }));

  // Add autos here
  // Default

  auto_chooser_.SetDefaultOption("drive_auto",
                                 drive_auto_.get());
  
  // auto_chooser_.AddOption("five_piece_auto", five_piece_auto_.get());

  // Other options
  frc::SmartDashboard::PutData(&auto_chooser_);

  // Verify robot hardware
  VerifyHardware();

  // Set initial target for all subsystems to zero.
  for (auto subsystem : container_.all_subsystems_) {
    subsystem->SetTargetZero();
  }

  // Report to driver station that robot is ready
  Log("\n********** Funky robot initialized **********\n");
  HAL_ObserveUserProgramStarting();

  for (;;) {
    frc::DriverStation::RefreshData();
    next_loop_time_ += kPeriod;

    // Set new notifier time
    int32_t status = 0;
    HAL_UpdateNotifierAlarm(notifier_, next_loop_time_.to<uint64_t>(), &status);
    FRC_CheckErrorStatus(status, "{}", "UpdateNotifierAlarm");

    // Wait for notifier
    auto time = HAL_WaitForNotifierAlarm(notifier_, &status);
    FRC_CheckErrorStatus(status, "{}", "WaitForNotifierAlarm");


    if (time == 0 || status != 0) {
      break;
    }

    // Start loop timing
    auto loop_start_time = frc846::wpilib::CurrentFPGATime();

    // Get current control mode
    frc::DSControlWord word{};
    Mode mode = Mode::kNone;
    if (word.IsDisabled()) {
      HAL_ObserveUserProgramDisabled();
      mode = Mode::kDisabled;
    } else if (word.IsAutonomous()) {
      HAL_ObserveUserProgramAutonomous();
      mode = Mode::kAutonomous;
    } else if (word.IsTeleop()) {
      HAL_ObserveUserProgramTeleop();
      mode = Mode::kTeleop;
    } else if (word.IsTest()) {
      HAL_ObserveUserProgramTest();
      mode = Mode::kTest;
    }

    // If mode changed
    if (last_mode_ != mode) {
      if (mode == Mode::kDisabled) {
        // Clear command scheduler
        Log("Clearing command scheduler");
        frc2::CommandScheduler::GetInstance().CancelAll();
        frc::EventLoop loop;
        loop.Clear();
      } else if (mode == Mode::kAutonomous) {

        // Get and run selected auto command
        auto_command_ = auto_chooser_.GetSelected();

        if (auto_command_ != nullptr) {
          Log("Running auto: {}", auto_command_->GetName());
          auto_command_->Schedule();
        } else {
          Error("Auto command null!");
        }
      } else if (mode == Mode::kTeleop) {
        // Cancel auto command and setup teleop defaults/triggers
        if (auto_command_ != nullptr) {
          Log("Cancelling auto command");
          auto_command_->Cancel();
          auto_command_ = nullptr;
        }

        Log("Setting up teleop default/triggers");
        InitTeleopDefaults();
        InitTeleopTriggers();
      } else if (mode == Mode::kTest) {
        /// Cancel auto command and setup Test defaults/triggers
        if (auto_command_ != nullptr) {
          Log("Cancelling auto command");
          auto_command_->Cancel();
          auto_command_ = nullptr;
        }

        Log("Setting up test default/triggers");
        // Different defaults as Teleop
        InitTestDefaults();
        // Same triggers as Teleop
        InitTeleopTriggers();
      }

      last_mode_ = mode;
    }

    // Update subsystem readings
    for (auto subsystem : container_.all_subsystems_) {
      subsystem->UpdateReadings();
    }

    // Tick command scheduler
    frc2::CommandScheduler::GetInstance().Run();

    // Update subsystem hardware
    for (auto subsystem : container_.all_subsystems_) {
      subsystem->UpdateHardware();
    }

    // Zero
    if (frc::RobotController::GetUserButton()) {
      container_.drivetrain_.ZeroCancoders();
    }

    if (!homing_switch_.Get() && word.IsDisabled()) {
      container_.pivot_.ZeroSubsystem();
      container_.telescope_.ZeroSubsystem();
      container_.wrist_.ZeroSubsystem();
    }

    // Update dashboards
    frc::SmartDashboard::UpdateValues();
    frc::Shuffleboard::Update();

    // Update graphs
    time_remaining_graph_.Graph(frc::DriverStation::GetMatchTime().to<int>());

    warnings_graph_.Graph(frc846::Loggable::GetWarnCount());
    errors_graph_.Graph(frc846::Loggable::GetErrorCount());

    can_usage_graph_.Graph(
        frc::RobotController::GetCANStatus().percentBusUtilization * 100);

    auto loop_time = frc846::wpilib::CurrentFPGATime() - loop_start_time;
    loop_time_graph_.Graph(frc846::wpilib::CurrentFPGATime() - loop_start_time);

    // Check loop time
    if (loop_time > kPeriod * 2) {
      Warn("Bad loop overrun: {} (loop period: {})",
           loop_time.convert<units::millisecond>(), kPeriod);
    }
  }
}

void FunkyRobot::EndCompetition() {
  Log("\n********** Robot code ending **********\n");
}

void FunkyRobot::InitTeleopDefaults() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
}

void FunkyRobot::InitTeleopTriggers() {
  // WARNING: Driver left_bumper & right_bumper already taken
  frc2::Trigger drivetrain_zero_bearing_trigger{
      [&] { return container_.driver_.readings().back_button; }};

  frc2::Trigger shintake_test_in_trigger{
      [&] { return container_.driver_.readings().right_trigger; }};

  frc2::Trigger shintake_test_out_trigger{
      [&] { return container_.driver_.readings().left_trigger; }};

  frc2::Trigger tele_test_up_trigger{
      [&] { return container_.driver_.readings().b_button; }};

  frc2::Trigger tele_test_down_trigger{
      [&] { return container_.driver_.readings().a_button; }};

  frc2::Trigger pivot_test_up_trigger{
      [&] { return container_.driver_.readings().y_button; }};

  frc2::Trigger pivot_test_down_trigger{
      [&] { return container_.driver_.readings().x_button; }};

  frc2::Trigger wrist_test_up_trigger{
      [&] { return container_.driver_.readings().pov == frc846::XboxPOV::kUp; }};

  frc2::Trigger wrist_test_down_trigger{
      [&] { return container_.driver_.readings().pov == frc846::XboxPOV::kDown; }};

  frc2::Trigger stow_trigger{
      [&] { return container_.driver_.readings().pov == frc846::XboxPOV::kLeft; }};

  frc2::Trigger intake_deploy_trigger{
      [&] { return container_.driver_.readings().pov == frc846::XboxPOV::kRight; }};
  

  // // Bind Triggers to commands
  drivetrain_zero_bearing_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.drivetrain_.ZeroBearing();
      }).ToPtr());

  shintake_test_in_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.shintake_.SetTarget(container_.shintake_.MakeTarget(true, false, 0.0_tps));
      }).ToPtr());

  shintake_test_in_trigger.OnFalse(
      frc2::InstantCommand([this] {
        container_.shintake_.SetTarget(container_.shintake_.ZeroTarget());
      }).ToPtr());

  shintake_test_out_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.shintake_.SetTarget(container_.shintake_.MakeTarget(false, true, container_.shintake_.shooter_speed_.value()));
      }).ToPtr());

  shintake_test_out_trigger.OnFalse(
      frc2::InstantCommand([this] {
        container_.shintake_.SetTarget(container_.shintake_.ZeroTarget());
      }).ToPtr());

  pivot_test_up_trigger.WhileTrue(
    frc2::InstantCommand([this] {
        PivotTarget t = container_.pivot_.GetTarget();
        t.pivot_output = 0.1;
        container_.pivot_.SetTarget(t);
    }).ToPtr());

  pivot_test_up_trigger.OnFalse(
    frc2::InstantCommand([this] {
        PivotTarget t = container_.pivot_.GetTarget();
        t.pivot_output = container_.pivot_.readings().pivot_position;
        container_.pivot_.SetTarget(t);
    }).ToPtr());

  pivot_test_down_trigger.WhileTrue(
    frc2::InstantCommand([this] {
        PivotTarget t = container_.pivot_.GetTarget();
        t.pivot_output = -0.1;
        container_.pivot_.SetTarget(t);
    }).ToPtr());

  pivot_test_down_trigger.OnFalse(
    frc2::InstantCommand([this] {
        PivotTarget t = container_.pivot_.GetTarget();
        t.pivot_output = container_.pivot_.readings().pivot_position;
        container_.pivot_.SetTarget(t);
    }).ToPtr());

  tele_test_up_trigger.WhileTrue(
    frc2::InstantCommand([this] {
        TelescopeTarget t = container_.telescope_.GetTarget();
        t.extension = 0.1;
        container_.telescope_.SetTarget(t);
    }).ToPtr());

  tele_test_up_trigger.OnFalse(
    frc2::InstantCommand([this] {
        TelescopeTarget t = container_.telescope_.GetTarget();
        t.extension = container_.telescope_.readings().extension;
        container_.telescope_.SetTarget(t);
    }).ToPtr());

  tele_test_down_trigger.WhileTrue(
    frc2::InstantCommand([this] {
        TelescopeTarget t = container_.telescope_.GetTarget();
        t.extension = -0.1;
        container_.telescope_.SetTarget(t);
    }).ToPtr());

  tele_test_down_trigger.OnFalse(
    frc2::InstantCommand([this] {
        TelescopeTarget t = container_.telescope_.GetTarget();
        t.extension = container_.telescope_.readings().extension;
        container_.telescope_.SetTarget(t);
    }).ToPtr());

  wrist_test_up_trigger.WhileTrue(
    frc2::InstantCommand([this] {
        WristTarget t = container_.wrist_.GetTarget();
        t.wrist_output = 0.1;
        container_.wrist_.SetTarget(t);
    }).ToPtr());

  wrist_test_up_trigger.OnFalse(
    frc2::InstantCommand([this] {
        WristTarget t = container_.wrist_.GetTarget();
        t.wrist_output = container_.wrist_.readings().wrist_position;
        container_.wrist_.SetTarget(t);
    }).ToPtr());

  wrist_test_down_trigger.WhileTrue(
    frc2::InstantCommand([this] {
        WristTarget t = container_.wrist_.GetTarget();
        t.wrist_output = -0.1;
        container_.wrist_.SetTarget(t);
    }).ToPtr());

  wrist_test_down_trigger.OnFalse(
    frc2::InstantCommand([this] {
        WristTarget t = container_.wrist_.GetTarget();
        t.wrist_output = container_.wrist_.readings().wrist_position;
        container_.wrist_.SetTarget(t);
    }).ToPtr());

  stow_trigger.OnTrue(StowCommand{container_}.ToPtr());
  // intake_deploy_trigger.OnTrue(DeployIntakeCommand{container_}.ToPtr());  
}

void FunkyRobot::InitTestDefaults() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
}

void FunkyRobot::VerifyHardware() {
  Log("Verifying hardware...");
  for (auto subsystem : container_.all_subsystems_) {
    bool ok = subsystem->VerifyHardware();
    if (!ok) {
      subsystem->Error("Failed hardware verification!!");
    }
  }
  Log("Done verifying hardware");
}
