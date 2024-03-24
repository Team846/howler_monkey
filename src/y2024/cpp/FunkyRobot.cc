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
#include <cameraserver/CameraServer.h>

#include "frc/DataLogManager.h"
#include "frc2/command/ParallelDeadlineGroup.h"
#include "frc2/command/WaitCommand.h"
#include "frc846/loggable.h"
#include "frc846/other/sendable_callback.h"
#include "frc846/wpilib/time.h"
#include "frc846/other/xbox.h"
#include "commands/teleop/drive_command.h"
#include "commands/teleop/teleop_positioning_command.h"
#include "commands/follow_trajectory_command.h"
#include "commands/zero/zero_wrist_command.h"
#include "subsystems/scorer.h"
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

  auto_chooser_.AddOption("drive_auto",
                                 drive_auto_.get());
  
  auto_chooser_.SetDefaultOption("five_piece_auto_blue", five_piece_auto_blue.get());
  auto_chooser_.AddOption("five_piece_auto_red", five_piece_auto_red.get());
  auto_chooser_.AddOption("four_piece_auto_lr", four_piece_auto_lr.get());
  auto_chooser_.AddOption("four_piece_auto_rl", four_piece_auto_rl.get());
  auto_chooser_.AddOption("three_piece_source_side_auto_blue", three_piece_source_auto_blue.get());
  auto_chooser_.AddOption("three_piece_source_side_auto_red", three_piece_source_auto_red.get());
  auto_chooser_.AddOption("one_piece_auto", one_piece_auto_.get());
  // auto_chooser_.AddOption("testing_routine", testing_routine_.get());

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
      coast_counter_ = 500;
    }

    if (coast_counter_ == 1) {
      container_.pivot_.Brake();
      container_.telescope_.Brake();
      container_.wrist_.Brake();
      coast_counter_ -= 1;
    } else if (coast_counter_ > 0) {
      coast_counter_ -= 1;
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
  container_.pivot_.SetDefaultCommand(TeleopPositioningCommand{container_});
}

void FunkyRobot::InitTeleopTriggers() {
  // WARNING: Driver left_bumper & right_bumper already taken
  frc2::Trigger drivetrain_zero_bearing_trigger{
      [&] { return container_.driver_.readings().back_button; }};


  frc2::Trigger on_piece_trigger{
      [&] { return frc846::util::ShareTables::GetBoolean("scorer_has_piece"); }};

  frc2::Trigger scorer_in_trigger{
      [&] { return container_.driver_.readings().left_trigger || container_.driver_.readings().x_button; }};

  frc2::Trigger scorer_in_source_trigger{
      [&] { return container_.driver_.readings().x_button; }};


  frc2::Trigger scorer_pass_trigger{
      [&] { return container_.driver_.readings().a_button; }};

  frc2::Trigger scorer_spin_up_trigger{
      [&] { return container_.driver_.readings().right_trigger 
        || container_.driver_.readings().y_button
          || container_.operator_.readings().right_stick_x > 0.5 
            || container_.operator_.readings().right_stick_y > 0.5; }};

  frc2::Trigger scorer_eject_trigger{
      [&] { return (container_.operator_.readings().right_bumper) || (
            !(container_.driver_.readings().right_trigger || container_.driver_.readings().y_button) && 
              container_.pivot_.readings().pivot_position > 40_deg &&
                container_.driver_.readings().right_bumper
        ); }};

  frc2::Trigger scorer_out_trigger{
      [&] { return ((container_.driver_.readings().right_bumper
        && (container_.driver_.readings().right_trigger || container_.driver_.readings().y_button) && 
                std::abs(container_.scorer_.readings().kLeftErrorPercent) < 0.35)
      ); }};
  
  frc2::Trigger scorer_lodge_trigger{
      [&] {return ((container_.operator_.readings().pov == frc846::XboxPOV::kLeft));}};

    //87, 157, 3.5

  // frc2::Trigger scorer_out_trigger{
  //     [&] { return ((container_.driver_.readings().right_bumper && (std::abs(container_.scorer_.readings().kLeftErrorPercent) < 0.2 
  //       || container_.driver_.readings().left_bumper)) || 
  //         container_.operator_.readings().pov == frc846::XboxPOV::kLeft
  //     ); }};

  frc2::Trigger scorer_roller_in_trigger{
      [&] { return (container_.operator_.readings().pov == frc846::XboxPOV::kRight); }};

  frc2::Trigger zero_wrist_trigger{
      [&] { return container_.driver_.readings().b_button;}};

  frc2::Trigger amp_trigger{
      [&] {return container_.driver_.readings().left_bumper;}};
  
  // // Bind Triggers to commands
  drivetrain_zero_bearing_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.drivetrain_.ZeroBearing();
      }).ToPtr());

  scorer_in_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIntake));
      }).ToPtr());

  scorer_in_trigger.OnFalse(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());

  scorer_in_source_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIntake));
      }).ToPtr());
  
  scorer_in_source_trigger.OnFalse(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());

  scorer_spin_up_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kSpinUp));
      }).ToPtr());

  scorer_spin_up_trigger.OnFalse(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());


  scorer_lodge_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kLodge));
      }).ToPtr());

  scorer_lodge_trigger.OnFalse(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());
  
  scorer_pass_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kPass));
      }).ToPtr());

  scorer_pass_trigger.OnFalse(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kSpinUp));
      }).ToPtr());

  scorer_out_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kShoot));
      }).ToPtr());

  scorer_out_trigger.OnFalse(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());

  scorer_roller_in_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kRollerIn));
      }).ToPtr());

  scorer_roller_in_trigger.OnFalse(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());

  scorer_eject_trigger.WhileTrue(
    frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kRelease));
      }).ToPtr());

  scorer_eject_trigger.OnFalse(
      frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());

  on_piece_trigger.OnTrue(
    frc2::InstantCommand([this] {
        DriverTarget driver_target{};
        driver_target.rumble = true;
        container_.driver_.SetTarget(driver_target);
      }).WithTimeout(1_s).AndThen(
        frc2::InstantCommand([this] {
          DriverTarget driver_target{};
          driver_target.rumble = false;
          container_.driver_.SetTarget(driver_target);
        }).ToPtr()
      ));
  
  amp_trigger.OnTrue(
        frc2::InstantCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kRollerIn));
      }).WithTimeout(1_s).AndThen(
        frc2::InstantCommand([this] {
          container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
        }).ToPtr()

  ));
  
  zero_wrist_trigger.OnTrue(
    ZeroWristCommand{container_}.ToPtr()
  );
}

void FunkyRobot::InitTestDefaults() {
}

void FunkyRobot::InitTestTriggers() {

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
