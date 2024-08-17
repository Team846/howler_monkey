#include "commands/teleop/leds_command.h"

LEDsCommand::LEDsCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, LEDsCommand>{
          container, "leds_command"} {
  AddRequirements({&container_.leds_});
}

void LEDsCommand::OnInit() {}

void LEDsCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  LEDsState lstate;

  if (frc846::util::ShareTables::GetString("mode") == "kAutonomous") {
    lstate = LEDsState::kLEDSAutonomous;
  } else if (frc846::util::ShareTables::GetBoolean("zero sequence")) {
    lstate = LEDsState::kLEDSZeroing;
  } else if (container_.super_structure_.GetHasZeroed()) {
    lstate = LEDsState::kLEDSNotReady;
  } else if (frc846::util::ShareTables::GetString("mode") == "disabled") {
    lstate = LEDsState::kLEDSDisabled;
  } else if (ci_readings_.amping_leds) {
    lstate = LEDsState::kLEDSAmpingLeds;
  } else if (ci_readings_.coopertition_leds) {
    lstate = LEDsState::kLEDSCOOPLeds;
  } else if (ci_readings_.stageOfTrap != 0) {
    lstate = LEDsState::kLEDSClimbing;
  } else if (frc846::util::ShareTables::GetBoolean("ready_to_shoot")) {
    lstate = LEDsState::kLEDSReadyToShoot;
  } else if (ci_readings_.running_prep_shoot ||
             ci_readings_.running_super_shoot) {
    lstate = LEDsState::kLEDSPreparingShoot;
  } else if (frc846::util::ShareTables::GetBoolean("scorer_has_piece")) {
    lstate = LEDsState::kLEDSHasPiece;
  } else {
    lstate = LEDsState::kLEDSTeleop;
  }

  container_.leds_.SetTarget({lstate});
}

void LEDsCommand::OnEnd(bool interrupted) {}

bool LEDsCommand::IsFinished() { return false; }