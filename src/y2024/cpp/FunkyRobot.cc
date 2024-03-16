#include "FunkyRobot.h"

#include <frc/DSControlWord.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/RunCommand.h>
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

  // Add autos here
  // Default

  auto_chooser_.AddOption("drive_auto",
                                 drive_auto_.get());
  
  auto_chooser_.SetDefaultOption("four_piece_auto_lr", five_piece_auto_lr.get());
  auto_chooser_.AddOption("four_piece_auto_rl", five_piece_auto_rl.get());
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


        Log("Clearing command scheduler");
        frc2::CommandScheduler::GetInstance().CancelAll();
        frc::EventLoop loop;
        loop.Clear();

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
  frc2::Trigger drivetrain_zero_bearing_trigger{
      [&] { return container_.driver_.readings().back_button; }};


  frc2::Trigger on_piece_trigger{
      [&] { return frc846::util::ShareTables::GetBoolean("scorer_has_piece"); }};

  frc2::Trigger scorer_in_trigger{
      [&] { return container_.driver_.readings().left_trigger || container_.driver_.readings().x_button; }};

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
        && (container_.driver_.readings().right_trigger || container_.driver_.readings().y_button) && std::abs(container_.scorer_.readings().kLeftErrorPercent) < 0.35) || 
          container_.operator_.readings().pov == frc846::XboxPOV::kLeft
      ); }};

    //87, 157, 3.5

  // frc2::Trigger scorer_out_trigger{
  //     [&] { return ((container_.driver_.readings().right_bumper && (std::abs(container_.scorer_.readings().kLeftErrorPercent) < 0.2 
  //       || container_.driver_.readings().left_bumper)) || 
  //         container_.operator_.readings().pov == frc846::XboxPOV::kLeft
  //     ); }};

  frc2::Trigger scorer_manual_intake_trigger{
      [&] { return (container_.operator_.readings().pov == frc846::XboxPOV::kRight); }};
  
  // // Bind Triggers to commands
  drivetrain_zero_bearing_trigger.WhileTrue(
      frc2::RunCommand([this] {
        container_.drivetrain_.ZeroBearing();
      }).ToPtr());

  scorer_in_trigger.WhileTrue(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIntake));
      }).ToPtr());

  scorer_in_trigger.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());

  scorer_spin_up_trigger.WhileTrue(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kSpinUp));
      }).ToPtr());

  scorer_spin_up_trigger.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());

  scorer_out_trigger.WhileTrue(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kShoot));
      }).ToPtr());

  scorer_out_trigger.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());

  scorer_manual_intake_trigger.WhileTrue(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIntake));
      }).ToPtr());

  scorer_manual_intake_trigger.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());

  scorer_eject_trigger.WhileTrue(
    frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kRelease));
      }).ToPtr());

  scorer_eject_trigger.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
      }).ToPtr());

  on_piece_trigger.OnTrue(
    frc2::RunCommand([this] {
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
}

void FunkyRobot::InitTestDefaults() {
}

void FunkyRobot::InitTestTriggers() {
  std::cout<<"hi there"<<std::endl;
  frc2::Trigger FLS_forward{
      [&] { return container_.driver_.readings().left_stick_x>0.5;}};
  frc2::Trigger FLS_backward{
      [&] { return container_.driver_.readings().left_stick_x<-0.5;}};
  frc2::Trigger FLD_forward{
      [&] { return container_.driver_.readings().left_stick_y>0.5;}};
  frc2::Trigger FLD_backward{
      [&] { return container_.driver_.readings().left_stick_y<-0.5;}};

  frc2::Trigger FRS_forward{
      [&] { return container_.driver_.readings().right_stick_x>0.5;}};
  frc2::Trigger FRS_backward{
      [&] { return container_.driver_.readings().right_stick_x<-0.5;}};
  frc2::Trigger FRD_forward{
      [&] { return container_.driver_.readings().right_stick_y>0.5;}};
  frc2::Trigger FRD_backward{
      [&] { return container_.driver_.readings().right_stick_y<-0.5;}};

  frc2::Trigger BLS_forward{
      [&] { return container_.operator_.readings().left_stick_x>0.5;}};
  frc2::Trigger BLS_backward{
      [&] { return container_.operator_.readings().left_stick_x<-0.5;}};
  frc2::Trigger BLD_forward{
      [&] { return container_.operator_.readings().left_stick_y>0.5;}};
  frc2::Trigger BLD_backward{
      [&] { return container_.operator_.readings().left_stick_y<-0.5;}};

  frc2::Trigger BRS_forward{
      [&] { return container_.operator_.readings().right_stick_x>0.5;}};
  frc2::Trigger BRS_backward{
      [&] { return container_.operator_.readings().right_stick_x<-0.5;}};
  frc2::Trigger BRD_forward{
      [&] { return container_.operator_.readings().right_stick_y>0.5;}};
  frc2::Trigger BRD_backward{
      [&] { return container_.operator_.readings().right_stick_y<-0.5;}};

  frc2::Trigger pivot_forward{
      [&] { return container_.driver_.readings().y_button;}};
  frc2::Trigger pivot_backward{
      [&] { return container_.driver_.readings().x_button;}};

  frc2::Trigger telescope_forward{
      [&] { return container_.driver_.readings().b_button;}};
  frc2::Trigger telescope_backward{
      [&] { return container_.driver_.readings().a_button;}};

  frc2::Trigger wrist_forward{
      [&] { return container_.driver_.readings().pov==frc846::XboxPOV::kUp;}};
  frc2::Trigger wrist_backward{
      [&] { return container_.driver_.readings().pov==frc846::XboxPOV::kDown;}};

  frc2::Trigger shooter_one_forward{
      [&] { return container_.driver_.readings().right_trigger;}};
  frc2::Trigger shooter_one_backward{
      [&] { return container_.driver_.readings().left_trigger;}};

  frc2::Trigger shooter_two_forward{
      [&] { return container_.driver_.readings().right_bumper;}};
  frc2::Trigger shooter_two_backward{
      [&] { return container_.driver_.readings().left_bumper;}};

  frc2::Trigger intake_forward{
      [&] { return container_.driver_.readings().pov==frc846::XboxPOV::kLeft;}};
  frc2::Trigger intake_backward{
      [&] { return container_.driver_.readings().pov==frc846::XboxPOV::kRight;}};

  frc2::Trigger bracer_forward{
      [&] { return container_.driver_.readings().back_button;}};
  frc2::Trigger bracer_backward{
      [&] { return container_.driver_.readings().start_button;}};

  pivot_forward.WhileTrue(
    frc2::RunCommand([this] {
        container_.pivot_.SetTarget(container_.pivot_.MakeTarget(container_.pivot_.target_pivot_position+container_.pivot_.service_forward_increment.value()));
  }).ToPtr());
  pivot_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.pivot_.SetTarget(container_.pivot_.MakeTarget(container_.pivot_.target_pivot_position));
  }).ToPtr());
  pivot_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.pivot_.SetTarget(container_.pivot_.MakeTarget(container_.pivot_.target_pivot_position-container_.pivot_.service_backward_increment.value()));
  }).ToPtr());
  pivot_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.pivot_.SetTarget(container_.pivot_.MakeTarget(container_.pivot_.target_pivot_position));
  }).ToPtr());

  // Shooter_one
  shooter_one_forward.WhileTrue(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kScorerTest, container_.scorer_.service_shooter_forward_dc.value(), 0.0, 0.0));
  }).ToPtr());
  shooter_one_forward.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget({kIdle});
  }).ToPtr());
  shooter_one_backward.WhileTrue(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kScorerTest, container_.scorer_.service_shooter_backward_dc.value(), 0.0, 0.0));
  }).ToPtr());
  shooter_one_backward.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
  }).ToPtr());

  //Shooter_two
  shooter_two_forward.WhileTrue(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kScorerTest, 0.0, container_.scorer_.service_shooter_forward_dc.value(), 0.0));
  }).ToPtr());
  shooter_two_forward.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
  }).ToPtr());
  shooter_two_backward.WhileTrue(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kScorerTest, 0.0, container_.scorer_.service_shooter_backward_dc.value(), 0.0));
  }).ToPtr());
  shooter_two_backward.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
  }).ToPtr());

  //Intake
  intake_forward.WhileTrue(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kScorerTest, 0.0, 0.0, container_.scorer_.service_intake_forward_dc.value()));
  }).ToPtr());
  intake_forward.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
  }).ToPtr());
  intake_backward.WhileTrue(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kScorerTest, 0.0, 0.0, container_.scorer_.service_intake_backward_dc.value()));
  }).ToPtr());
  intake_backward.OnFalse(
      frc2::RunCommand([this] {
        container_.scorer_.SetTarget(container_.scorer_.MakeTarget(kIdle));
  }).ToPtr());

  //Telescope
  telescope_forward.WhileTrue(
    frc2::RunCommand([this] {
        container_.telescope_.SetTarget(container_.telescope_.MakeTarget(container_.telescope_.target_extension+container_.telescope_.service_forward_increment.value()));
  }).ToPtr());
  telescope_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.telescope_.SetTarget(container_.telescope_.MakeTarget(container_.telescope_.target_extension));
  }).ToPtr());
  telescope_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.telescope_.SetTarget(container_.telescope_.MakeTarget(container_.telescope_.target_extension-container_.telescope_.service_backward_increment.value()));
  }).ToPtr());
  telescope_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.telescope_.SetTarget(container_.telescope_.MakeTarget(container_.telescope_.target_extension));
  }).ToPtr());

  //Wrist
  wrist_forward.WhileTrue(
    frc2::RunCommand([this] {
        container_.wrist_.SetTarget(container_.wrist_.MakeTarget(container_.wrist_.target_wrist_position+container_.wrist_.service_forward_increment.value()));
  }).ToPtr());
  wrist_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.wrist_.SetTarget(container_.wrist_.MakeTarget(container_.wrist_.target_wrist_position));
  }).ToPtr());
  wrist_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.wrist_.SetTarget(container_.wrist_.MakeTarget(container_.wrist_.target_wrist_position-container_.wrist_.service_backward_increment.value()));
  }).ToPtr());
  wrist_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.wrist_.SetTarget(container_.wrist_.MakeTarget(container_.wrist_.target_wrist_position));
  }).ToPtr());

  //FL
  FLS_forward.WhileTrue(
    frc2::RunCommand([this] {
        std::cout<<"FLS_Forward"<<std::endl;;
        container_.drivetrain_.module_fl_.SetTarget(container_.drivetrain_.module_fl_.MakeTarget(0_fps, container_.drivetrain_.module_fl_.target_direction+container_.drivetrain_.service_steer_forward_increment.value(), kClosedLoop));
  }).ToPtr());
  FLS_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fl_.SetTarget(container_.drivetrain_.module_fl_.MakeTarget(0_fps, container_.drivetrain_.module_fl_.target_direction, kClosedLoop));
  }).ToPtr());
  FLS_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fl_.SetTarget(container_.drivetrain_.module_fl_.MakeTarget(0_fps, container_.drivetrain_.module_fl_.target_direction+container_.drivetrain_.service_steer_backward_increment.value(), kClosedLoop));
  }).ToPtr());
  FLS_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fl_.SetTarget(container_.drivetrain_.module_fl_.MakeTarget(0_fps, container_.drivetrain_.module_fl_.target_direction, kClosedLoop));
  }).ToPtr());
  FLD_forward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fl_.SetTarget(container_.drivetrain_.module_fl_.MakeTarget(container_.drivetrain_.service_drive_forward.value(), container_.drivetrain_.module_fl_.target_direction, kClosedLoop));
  }).ToPtr());
  FLD_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fl_.SetTarget(container_.drivetrain_.module_fl_.MakeTarget(0_fps, container_.drivetrain_.module_fl_.target_direction, kClosedLoop));
  }).ToPtr());
  FLD_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fl_.SetTarget(container_.drivetrain_.module_fl_.MakeTarget(container_.drivetrain_.service_drive_backward.value(), container_.drivetrain_.module_fl_.target_direction, kClosedLoop));
  }).ToPtr());
  FLD_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fl_.SetTarget(container_.drivetrain_.module_fl_.MakeTarget(0_fps, container_.drivetrain_.module_fl_.target_direction, kClosedLoop));
  }).ToPtr());
    //FR
  FRS_forward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fr_.SetTarget(container_.drivetrain_.module_fr_.MakeTarget(0_fps, container_.drivetrain_.module_fr_.target_direction+container_.drivetrain_.service_steer_forward_increment.value(), kClosedLoop));
  }).ToPtr());
  FRS_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fr_.SetTarget(container_.drivetrain_.module_fr_.MakeTarget(0_fps, container_.drivetrain_.module_fr_.target_direction, kClosedLoop));
  }).ToPtr());
  FRS_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fr_.SetTarget(container_.drivetrain_.module_fr_.MakeTarget(0_fps, container_.drivetrain_.module_fr_.target_direction+container_.drivetrain_.service_steer_backward_increment.value(), kClosedLoop));
  }).ToPtr());
  FRS_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fr_.SetTarget(container_.drivetrain_.module_fr_.MakeTarget(0_fps, container_.drivetrain_.module_fr_.target_direction, kClosedLoop));
  }).ToPtr());
  FRD_forward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fr_.SetTarget(container_.drivetrain_.module_fr_.MakeTarget(container_.drivetrain_.service_drive_forward.value(), container_.drivetrain_.module_fr_.target_direction, kClosedLoop));
  }).ToPtr());
  FRD_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fr_.SetTarget(container_.drivetrain_.module_fr_.MakeTarget(0_fps, container_.drivetrain_.module_fr_.target_direction, kClosedLoop));
  }).ToPtr());
  FRD_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fr_.SetTarget(container_.drivetrain_.module_fr_.MakeTarget(container_.drivetrain_.service_drive_backward.value(), container_.drivetrain_.module_fr_.target_direction, kClosedLoop));
  }).ToPtr());
  FRD_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_fr_.SetTarget(container_.drivetrain_.module_fr_.MakeTarget(0_fps, container_.drivetrain_.module_fr_.target_direction, kClosedLoop));
  }).ToPtr());
    //BL
  BLS_forward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_bl_.SetTarget(container_.drivetrain_.module_bl_.MakeTarget(0_fps, container_.drivetrain_.module_bl_.target_direction+container_.drivetrain_.service_steer_forward_increment.value(), kClosedLoop));
  }).ToPtr());
  BLS_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_bl_.SetTarget(container_.drivetrain_.module_bl_.MakeTarget(0_fps, container_.drivetrain_.module_bl_.target_direction, kClosedLoop));
  }).ToPtr());
  BLS_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_bl_.SetTarget(container_.drivetrain_.module_bl_.MakeTarget(0_fps, container_.drivetrain_.module_bl_.target_direction+container_.drivetrain_.service_steer_backward_increment.value(), kClosedLoop));
  }).ToPtr());
  BLS_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_bl_.SetTarget(container_.drivetrain_.module_bl_.MakeTarget(0_fps, container_.drivetrain_.module_bl_.target_direction, kClosedLoop));
  }).ToPtr());
  BLD_forward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_bl_.SetTarget(container_.drivetrain_.module_bl_.MakeTarget(container_.drivetrain_.service_drive_forward.value(), container_.drivetrain_.module_bl_.target_direction, kClosedLoop));
  }).ToPtr());
  BLD_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_bl_.SetTarget(container_.drivetrain_.module_bl_.MakeTarget(0_fps, container_.drivetrain_.module_bl_.target_direction, kClosedLoop));
  }).ToPtr());
  BLD_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_bl_.SetTarget(container_.drivetrain_.module_bl_.MakeTarget(container_.drivetrain_.service_drive_backward.value(), container_.drivetrain_.module_bl_.target_direction, kClosedLoop));
  }).ToPtr());
  BLD_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_bl_.SetTarget(container_.drivetrain_.module_bl_.MakeTarget(0_fps, container_.drivetrain_.module_bl_.target_direction, kClosedLoop));
  }).ToPtr());
    //BR
  BRS_forward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_br_.SetTarget(container_.drivetrain_.module_br_.MakeTarget(0_fps, container_.drivetrain_.module_br_.target_direction+container_.drivetrain_.service_steer_forward_increment.value(), kClosedLoop));
  }).ToPtr());
  BRS_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_br_.SetTarget(container_.drivetrain_.module_br_.MakeTarget(0_fps, container_.drivetrain_.module_br_.target_direction, kClosedLoop));
  }).ToPtr());
  BRS_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_br_.SetTarget(container_.drivetrain_.module_br_.MakeTarget(0_fps, container_.drivetrain_.module_br_.target_direction+container_.drivetrain_.service_steer_backward_increment.value(), kClosedLoop));
  }).ToPtr());
  BRS_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_br_.SetTarget(container_.drivetrain_.module_br_.MakeTarget(0_fps, container_.drivetrain_.module_br_.target_direction, kClosedLoop));
  }).ToPtr());
  BRD_forward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_br_.SetTarget(container_.drivetrain_.module_br_.MakeTarget(container_.drivetrain_.service_drive_forward.value(), container_.drivetrain_.module_br_.target_direction, kClosedLoop));
  }).ToPtr());
  BRD_forward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_br_.SetTarget(container_.drivetrain_.module_br_.MakeTarget(0_fps, container_.drivetrain_.module_br_.target_direction, kClosedLoop));
  }).ToPtr());
  BRD_backward.WhileTrue(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_br_.SetTarget(container_.drivetrain_.module_br_.MakeTarget(container_.drivetrain_.service_drive_backward.value(), container_.drivetrain_.module_br_.target_direction, kClosedLoop));
  }).ToPtr());
  BRD_backward.OnFalse(
    frc2::RunCommand([this] {
        container_.drivetrain_.module_br_.SetTarget(container_.drivetrain_.module_br_.MakeTarget(0_fps, container_.drivetrain_.module_br_.target_direction, kClosedLoop));
  }).ToPtr());
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
