#include "frc846/robot/GenericRobot.h"

#include <cameraserver/CameraServer.h>
#include <frc/DSControlWord.h>
#include <frc/DataLogManager.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "frc2/command/ParallelDeadlineGroup.h"
#include "frc2/command/WaitCommand.h"
#include "frc846/base/loggable.h"
#include "frc846/ntinf/ntaction.h"
#include "frc846/robot/GenericCommand.h"
#include "frc846/wpilib/time.h"

namespace frc846::robot {

GenericRobot::GenericRobot(GenericRobotContainer* container)
    : frc846::base::Loggable{"Robot"}, generic_robot_container_{container} {
  next_loop_time_ = frc846::wpilib::CurrentFPGATime();

  int32_t status = 0x00;
  notifier_ = HAL_InitializeNotifier(&status);
  FRC_CheckErrorStatus(status, "{}", "InitializeNotifier");

  HAL_SetNotifierName(notifier_, "Robot", &status);
}

GenericRobot::~GenericRobot() {
  int32_t status = 0x00;
  HAL_StopNotifier(notifier_, &status);
  HAL_CleanNotifier(notifier_, &status);
}

void GenericRobot::StartCompetition() {
  // Silence warnings related to missing joystick
  // (Doesn't do anything when connected to FMS)

  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // Disable live window
  frc::LiveWindow::DisableAllTelemetry();

  // Start Data Log Manager
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  frc846::base::FunkyLogSystem::Start(20000);

  // Add dashboard buttons

  frc::SmartDashboard::PutData(
      "verify_hardware",
      new frc846::ntinf::NTAction([this] { VerifyHardware(); }));

  frc::SmartDashboard::PutData(
      "get_prune_list", new frc846::ntinf::NTAction([this] {
        for (const std::string& x : robotStore.GetPruneList()) {
          Log("Key {} found in prune list.", x);
        }
      }));

  frc::SmartDashboard::PutData(
      "prune_prefs", new frc846::ntinf::NTAction([this] {
        for (const std::string& x : robotStore.GetPruneList()) {
          Log("Pruning key {}.", x);
        }
        robotStore.Prune();
      }));

  // Verify robot hardware
  VerifyHardware();

  // Setup all subsystems and set initial targets to zero.
  generic_robot_container_->Setup();
  generic_robot_container_->ZeroTargets();

  OnInitialize();

  // Report to driver station that robot is ready
  Log("\n********** Robot initialized **********\n");
  HAL_ObserveUserProgramStarting();

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
        std::string option_name = auto_chooser_.GetSelected();
        auto_command_ = autos_[option_name];

        if (auto_command_ != nullptr) {
          Log("Running auto: {}", option_name);

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
        InitTeleop();
      } else if (mode == Mode::kTest) {
        // Cancel auto command and setup Test defaults/triggers
        if (auto_command_ != nullptr) {
          Log("Cancelling auto command");
          auto_command_->Cancel();
          auto_command_ = nullptr;
        }

        Log("Setting up test default/triggers");
        InitTest();
      }

      last_mode_ = mode;
    }

    OnPeriodic();

    // Update subsystem readings
    generic_robot_container_->UpdateReadings();

    // Tick command scheduler
    frc2::CommandScheduler::GetInstance().Run();

    // Update subsystem hardware
    generic_robot_container_->UpdateHardware();

    // Update dashboards
    frc::SmartDashboard::UpdateValues();
    frc::Shuffleboard::Update();

    robotStore.Refresh();

    // Update graphs
    time_remaining_graph_.Graph(frc::DriverStation::GetMatchTime().to<int>());

    warnings_graph_.Graph(frc846::base::Loggable::GetWarnCount());
    errors_graph_.Graph(frc846::base::Loggable::GetErrorCount());

    can_usage_graph_.Graph(
        frc::RobotController::GetCANStatus().percentBusUtilization * 0x64);

    auto loop_time = frc846::wpilib::CurrentFPGATime() - loop_start_time;
    loop_time_graph_.Graph(frc846::wpilib::CurrentFPGATime() - loop_start_time);

    // Check loop time
    if (loop_time > kPeriod * 0x03) {
      Warn("Loop overrun: {} (loop period: {})",
           loop_time.convert<units::millisecond>(), kPeriod);
    }
  }
}

void GenericRobot::EndCompetition() {
  Log("\n********** Robot code ending **********\n");
}

void GenericRobot::VerifyHardware() {
  generic_robot_container_->VerifyHardware();
}

void GenericRobot::AddAuto(std::string name, frc2::Command* command) {
  auto_chooser_.AddOption(name, name);
  autos_[name] = command;
  frc::SmartDashboard::PutData(&auto_chooser_);
  frc::SmartDashboard::UpdateValues();
}

};  // namespace frc846::robot