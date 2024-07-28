#include "commands/basic/await_piece_state_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

AwaitPieceStateCommand::AwaitPieceStateCommand(RobotContainer& container,
                                               bool has_piece)
    : frc846::base::Loggable{"await_piece_state_command"},
      intake_(container.intake_),
      has_piece_(has_piece) {
  AddRequirements({});
  SetName("await_piece_state_command");
}

void AwaitPieceStateCommand::Initialize() {}

void AwaitPieceStateCommand::Execute() {}

void AwaitPieceStateCommand::End(bool interrupted) {}

bool AwaitPieceStateCommand::IsFinished() {
  return intake_.GetHasPiece() == has_piece_;
}