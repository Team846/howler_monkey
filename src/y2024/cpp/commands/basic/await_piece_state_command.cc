#include "commands/basic/await_piece_state_command.h"

AwaitPieceStateCommand::AwaitPieceStateCommand(RobotContainer& container,
                                               bool has_piece)
    : frc846::robot::GenericCommand<
          RobotContainer, AwaitPieceStateCommand>{container,
                                                  "await_piece_state_command"},
      target_has_piece_(has_piece) {
  AddRequirements({});
}

void AwaitPieceStateCommand::OnInit() {}

void AwaitPieceStateCommand::Periodic() {}

void AwaitPieceStateCommand::OnEnd(bool interrupted) {}

bool AwaitPieceStateCommand::IsFinished() {
  return container_.intake_.GetHasPiece() == target_has_piece_;
}