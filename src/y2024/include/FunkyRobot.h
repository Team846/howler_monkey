#pragma once

#include <frc/DigitalInput.h>

#include "autos/GenericAuto.h"
#include "frc846/ntinf/fstore.h"
#include "frc846/ntinf/pref.h"
#include "frc846/robot/GenericRobot.h"
#include "subsystems/robot_container.h"

class FunkyRobot : public frc846::robot::GenericRobot {
 public:
  FunkyRobot();

  void OnInitialize() override;

  void OnDisable() override;

  void OnPeriodic() override;

  void InitTeleop() override;
  void InitTest() override;

 private:
  RobotContainer container_;

  frc::DigitalInput homing_switch_{0};
  frc::DigitalInput coasting_switch_{1};

  frc846::ntinf::Pref<units::second_t> coasting_time_{*this, "coasting_time",
                                                      7.5_s};

  units::second_t stop_coast_time_;
};
