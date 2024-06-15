#include "commands/teleop/leds_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"

LEDsCommand::LEDsCommand(RobotContainer &container)
    : control_input_(container.control_input_),
      leds_(container.leds_),
      super_(container.super_structure_) {
  AddRequirements({&leds_});
  SetName("leds_command");
}

void LEDsCommand::Execute() {
  ControlInputReadings ci_readings_{control_input_.readings()};

  LEDsState lstate;

  if (frc846::util::ShareTables::GetString("mode") == "kAutonomous") {
    lstate = LEDsState::kLEDSAutonomous;
  } else if (frc846::util::ShareTables::GetBoolean("zero sequence")) {
    lstate = LEDsState::kLEDSZeroing;
  } else if (!super_.wrist_->GetHasZeroed()) {
    lstate = LEDsState::kLEDSNotReady;
  } else if (frc846::util::ShareTables::GetString("mode") == "disabled") {
    lstate = LEDsState::kLEDSDisabled;
  } else if (ci_readings_.amping_leds) {
    lstate = LEDsState::kLEDSAmpingLeds;
  } else if (ci_readings_.coopertition_leds) {
    lstate = LEDsState::kLEDSCOOPLeds;
  } else if (ci_readings_.running_prep_shoot ||
             ci_readings_.running_super_shoot) {
    lstate = LEDsState::kLEDSPreparingShoot;
  } else if (frc846::util::ShareTables::GetBoolean("scorer_has_piece")) {
    lstate = LEDsState::kLEDSHasPiece;
  } else {
    lstate = LEDsState::kLEDSTeleop;
  }

  leds_.SetTarget({lstate});
}

bool LEDsCommand::IsFinished() { return false; }