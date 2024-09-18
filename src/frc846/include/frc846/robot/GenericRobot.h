#pragma once

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <hal/Types.h>
#include <units/time.h>

#include "frc846/ntinf/fstore.h"
#include "frc846/ntinf/grapher.h"
#include "frc846/ntinf/pref.h"
#include "frc846/robot/GenericRobotContainer.h"

namespace frc846::robot {

enum Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

class GenericRobot : public frc::RobotBase, public frc846::base::Loggable {
 public:
  static constexpr auto kPeriod = 20_ms;  // 50hz

  GenericRobot(GenericRobotContainer* container);

  ~GenericRobot() override;

  void StartCompetition() override final;
  void EndCompetition() override final;

  virtual void OnInitialize() = 0;

  virtual void OnDisable() = 0;

  virtual void OnPeriodic() = 0;

  virtual void InitTeleop() = 0;
  virtual void InitTest() = 0;

  void VerifyHardware();

  void AddAutos(frc2::Command* defaultOption,
                std::vector<frc2::Command*> otherOptions);

 private:
  hal::Handle<HAL_NotifierHandle> notifier_;
  units::microsecond_t next_loop_time_;

  Mode last_mode_;

 private:
  GenericRobotContainer* generic_robot_container_;

  frc846::ntinf::Grapher<int> time_remaining_graph_{*this, "time"};

  frc846::ntinf::Grapher<int> warnings_graph_{*this, "warnings"};
  frc846::ntinf::Grapher<int> errors_graph_{*this, "errors"};

  frc846::ntinf::Grapher<double> can_usage_graph_{*this, "CAN_usage"};
  frc846::ntinf::Grapher<units::millisecond_t> loop_time_graph_{*this,
                                                                "loop_time"};

  frc2::Command* auto_command_ = nullptr;
  frc::SendableChooser<frc2::Command*> auto_chooser_;

  frc846::ntinf::FunkyStore robotStore{};
};

};  // namespace frc846::robot
