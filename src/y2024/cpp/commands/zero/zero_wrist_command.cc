#include "commands/zero/zero_wrist_command.h"

#include "subsystems/wrist.h"

ZeroWristCommand::ZeroWristCommand(RobotContainer& container)
    : wrist_(container.wrist_), 
      super_(container.super_structure_) {
  AddRequirements({&wrist_});
  SetName("wrist_zero_command");
  last_pos_ = frc846::util::ShareTables::GetDouble("wrist_position");
  is_done_ = false;
  loops = 1;
}

void ZeroWristCommand::Execute() {
    wrist_.SetTarget(wrist_.MakeTarget(wrist_.wrist_home_speed_.value()));

    if (loops % 25 == 0) {
        last_pos_ = frc846::util::ShareTables::GetDouble("wrist_position");
    } else if (frc846::util::ShareTables::GetDouble("wrist_position") >= last_pos_){
        std::cout << "wrist stopped moving. now zeroing" << std::endl;
        wrist_.ZeroSubsystem();
        is_done_ = true;
    }
    
    if (loops > 250) {
        std::cout << "wrist time limit met. now ending zero command" << std:: endl;
        is_done_ = true;
    }

    loops++;
}

bool ZeroWristCommand::IsFinished() {
    return is_done_;
}