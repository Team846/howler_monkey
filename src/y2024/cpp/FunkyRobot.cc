#include "FunkyRobot.h"

#include <cameraserver/CameraServer.h>
#include <frc/DSControlWord.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "commands/basic/amp_command.h"
#include "commands/basic/deploy_intake_command.h"
#include "commands/basic/idle_command.h"
#include "commands/basic/shoot_command.h"
#include "commands/basic/spin_up_command.h"
#include "commands/basic/stow_command.h"
#include "commands/follow_trajectory_command.h"
#include "commands/teleop/bracer_command.h"
#include "commands/teleop/control_input_command.h"
#include "commands/teleop/drive_command.h"
#include "commands/teleop/leds_command.h"
#include "commands/teleop/operator_control.h"
#include "control_triggers.h"
#include "frc/DataLogManager.h"
#include "frc2/command/ParallelDeadlineGroup.h"
#include "frc2/command/WaitCommand.h"
#include "frc846/loggable.h"
#include "frc846/other/sendable_callback.h"
#include "frc846/other/xbox.h"
#include "frc846/wpilib/time.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/pivot.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/hardware/telescope.h"
#include "subsystems/hardware/wrist.h"

FunkyRobot::FunkyRobot() : frc846::Loggable{"funky_robot"} {
  next_loop_time_ = frc846::wpilib::CurrentFPGATime();

  int32_t status = 0x00;
  notifier_ = HAL_InitializeNotifier(&status);
  FRC_CheckErrorStatus(status, "{}", "InitializeNotifier");

  HAL_SetNotifierName(notifier_, "FunkyRobot", &status);
}

FunkyRobot::~FunkyRobot() {
  int32_t status = 0x00;
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

  // std::thread visionThread(VisionThread);
  // visionThread.detach();

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

  frc::SmartDashboard::PutData(
      "verify_hardware",
      new frc846::other::SendableCallback([this] { VerifyHardware(); }));

  frc::SmartDashboard::PutData("zero_subsystems",
                               new frc846::other::SendableCallback([this] {
                                 container_.pivot_.ZeroSubsystem();
                                 container_.telescope_.ZeroSubsystem();
                                 container_.wrist_.ZeroSubsystem();
                                 container_.leds_.ZeroSubsystem();
                                 container_.drivetrain_.ZeroBearing();
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

  // Add autos here
  // Default

  auto_chooser_.AddOption("drive_auto", drive_auto_.get());

  auto_chooser_.SetDefaultOption("four_piece_auto_lr",
                                 four_piece_auto_lr.get());
  auto_chooser_.AddOption("four_piece_auto_rl", four_piece_auto_rl.get());

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

  // container_.leds_.SetDefaultCommand(LEDsCommand{container_});

  for (;;) {
    frc::DriverStation::RefreshData();
    next_loop_time_ += kPeriod;

    // Set new notifier time
    int32_t status = 0x00;
    HAL_UpdateNotifierAlarm(notifier_, next_loop_time_.to<uint64_t>(), &status);
    FRC_CheckErrorStatus(status, "{}", "UpdateNotifierAlarm");

    // Wait for notifier
    auto time = HAL_WaitForNotifierAlarm(notifier_, &status);
    FRC_CheckErrorStatus(status, "{}", "WaitForNotifierAlarm");

    if (time == 0x00 || status != 0x00) {
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
      frc846::util::ShareTables::SetString("mode", "disabled");
    } else if (word.IsAutonomous()) {
      HAL_ObserveUserProgramAutonomous();
      mode = Mode::kAutonomous;
      frc846::util::ShareTables::SetString("mode", "kAutonomous");
    } else if (word.IsTeleop()) {
      HAL_ObserveUserProgramTeleop();
      mode = Mode::kTeleop;
      frc846::util::ShareTables::SetString("mode", "kTeleop");
    } else if (word.IsTest()) {
      HAL_ObserveUserProgramTest();
      mode = Mode::kTest;
      frc846::util::ShareTables::SetString("mode", "kTeleop");
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

        container_.pivot_.Brake();
        container_.telescope_.Brake();
        container_.wrist_.Brake();

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
        InitTestTriggers();
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
      std::cout << "Zeroing all subsystems" << std::endl;
      frc846::util::ShareTables::SetBoolean("zero sequence", true);
      container_.pivot_.ZeroSubsystem();
      container_.telescope_.ZeroSubsystem();
      container_.wrist_.ZeroSubsystem();
      container_.leds_.ZeroSubsystem();
      container_.drivetrain_.ZeroBearing();
    }
    if (!coasting_switch_.Get() && word.IsDisabled()) {
      std::cout << "All Coast" << std::endl;
      container_.pivot_.Coast();
      container_.telescope_.Coast();
      container_.wrist_.Coast();
      coast_counter_ = 0x32;
    }

    if (coast_counter_ == 0x01) {
      container_.pivot_.Brake();
      container_.telescope_.Brake();
      container_.wrist_.Brake();
    }

    if (coast_counter_ > 0x00) {
      coast_counter_ -= 0x01;
    }

    // Update dashboards
    frc::SmartDashboard::UpdateValues();
    frc::Shuffleboard::Update();

    robotStore.Refresh();

    // Update graphs
    time_remaining_graph_.Graph(frc::DriverStation::GetMatchTime().to<int>());

    warnings_graph_.Graph(frc846::Loggable::GetWarnCount());
    errors_graph_.Graph(frc846::Loggable::GetErrorCount());

    can_usage_graph_.Graph(
        frc::RobotController::GetCANStatus().percentBusUtilization * 0x64);

    auto loop_time = frc846::wpilib::CurrentFPGATime() - loop_start_time;
    loop_time_graph_.Graph(frc846::wpilib::CurrentFPGATime() - loop_start_time);

    // Check loop time
    if (loop_time > kPeriod * 0x02) {
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
  container_.super_structure_.SetDefaultCommand(
      OperatorControlCommand{container_});
  container_.bracer_.SetDefaultCommand(BracerCommand{container_});
  container_.control_input_.SetDefaultCommand(ControlInputCommand{container_});
}

void FunkyRobot::InitTeleopTriggers() {
  frc2::Trigger drivetrain_zero_bearing_trigger{
      [&] { return container_.driver_.readings().back_button; }};

  drivetrain_zero_bearing_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.drivetrain_.ZeroBearing();
      }).ToPtr());

  frc2::Trigger on_piece_trigger{[&] {
    return frc846::util::ShareTables::GetBoolean("scorer_has_piece");
  }};

  on_piece_trigger.OnTrue(frc2::InstantCommand([this] {
                            DriverTarget driver_target{};
                            driver_target.rumble = true;
                            container_.driver_.SetTarget(driver_target);
                          })
                              .WithTimeout(1_s)
                              .AndThen(frc2::InstantCommand([this] {
                                         DriverTarget driver_target{};
                                         driver_target.rumble = false;
                                         container_.driver_.SetTarget(
                                             driver_target);
                                       }).ToPtr()));

  frc2::Trigger zero_wrist_trigger{
      [&] { return container_.operator_.readings().back_button; }};

  zero_wrist_trigger.OnTrue(frc2::InstantCommand([this] {
                              container_.wrist_.ZeroSubsystem();
                            }).ToPtr());

  ControlTriggerInitializer::InitTeleopTriggers(container_);
}

void FunkyRobot::InitTestDefaults() {}

void FunkyRobot::InitTestTriggers() {}

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
