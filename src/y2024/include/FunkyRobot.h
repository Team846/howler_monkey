#pragma once

#include <frc/DigitalInput.h>

#include "autos/drive_auto.h"
#include "autos/five_piece_auto.h"
#include "autos/one_piece_auto.h"
#include "frc846/fstore.h"
#include "frc846/robot/GenericRobot.h"
#include "frc846/util/pref.h"
#include "subsystems/robot_container.h"

class FunkyRobot : public frc846::robot::GenericRobot {
 public:
  FunkyRobot();

  void OnInitialize() override;

  void InitTeleop() override;
  void InitTest() override;

 private:
  RobotContainer container_;

  // Autos
  frc2::CommandPtr drive_auto_ = DriveAuto{container_}.ToPtr();

  frc2::CommandPtr five_piece_auto_red =
      FivePieceAuto{container_, false}.ToPtr();

  frc2::CommandPtr five_piece_auto_blue =
      FivePieceAuto{container_, true}.ToPtr();

  frc2::CommandPtr one_piece_auto_0 =
      OnePieceAuto{container_, -60_deg, "left red"}.ToPtr();
  frc2::CommandPtr one_piece_auto_1 =
      OnePieceAuto{container_, 60_deg, "right red"}.ToPtr();
  frc2::CommandPtr one_piece_auto_2 =
      OnePieceAuto{container_, -60_deg + 180_deg, "left blue"}.ToPtr();
  frc2::CommandPtr one_piece_auto_3 =
      OnePieceAuto{container_, 60_deg + 180_deg, "right blue"}.ToPtr();

  frc::DigitalInput homing_switch_{0};
  frc::DigitalInput coasting_switch_{1};

  int coast_counter_ = 0;
};
