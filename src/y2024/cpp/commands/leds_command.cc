#include "commands/leds_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

LEDsCommand::LEDsCommand(
    RobotContainer& container)
    : frc846::Loggable{"leds_command"}, leds_(container.leds_), 
      scorer_(container.scorer_), operator_(container.operator_) {
  AddRequirements({&leds_, &scorer_, &operator_});
  SetName("stow_command");
}

void LEDsCommand::Initialize() {
  Log("LED Command Initialize");
}

void LEDsCommand::Execute() {
  LEDsTarget target;

  target.has_zeroed = leds_.GetHasZeroed();
  //TODO check other subsystems?
  
  //TODO find better way to steal controls from operator
  if (leds_.is_initialized()) {
    target.kHumanPlayer =
        operator_.readings().left_trigger ? LEDsHumanPlayerState::kAmplify 
                                          : LEDsHumanPlayerState::kHumanPlayerIdle;
  }

  if (scorer_.is_initialized()){
    target.kPiece = scorer_.GetHasPiece() ? LEDsPieceState::kHasPiece 
                                                  : LEDsPieceState::kNoPieceIdle;
  }

  //TODO auto state
  leds_.SetTarget(target);
}

void LEDsCommand::End(bool interrupted) {
  Log("LED Command Finished");
}
