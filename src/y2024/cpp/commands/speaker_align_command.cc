#include "commands/speaker_align_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

#include "subsystems/field.h"

#include "frc846/util/share_tables.h"

SpeakerAlignCommand::SpeakerAlignCommand(
    RobotContainer& container, frc846::util::Vector2D<units::foot_t> shooting_point)
    : frc846::Loggable{"speaker_align_command"},
      drivetrain_(container.drivetrain_), 
          shoot_point(shooting_point) {
  AddRequirements({&drivetrain_});
  SetName("speaker_align_command");
}

void SpeakerAlignCommand::Initialize() {
  Log("Speaker Align Command Initialize");
}

void SpeakerAlignCommand::Execute() {
  auto current_x = shoot_point.x;
  auto current_y = shoot_point.y;
  auto target_x = field::points::kSpeaker(frc846::util::ShareTables::GetBoolean("is_red_side")).x;
  auto target_y = field::points::kSpeaker(frc846::util::ShareTables::GetBoolean("is_red_side")).y;

  auto drivetrain_target = drivetrain_.ZeroTarget();

  auto dist_x = target_x - current_x;
  auto dist_y = target_y - current_y;


  units::degree_t target_angle = units::math::atan2(dist_x, units::math::abs(dist_y));
      
  drivetrain_target.rotation = DrivetrainRotationPosition(target_angle);

  auto detect_angle = std::abs(((int)((target_angle.to<double>() 
    - drivetrain_.readings().pose.bearing.to<double>()))));


  std::cout << target_angle.to<double>() << std::endl;
  std::cout << drivetrain_.readings().pose.bearing.to<double>() << std::endl;
  is_done_ = detect_angle < 5 || (detect_angle > 355 && detect_angle < 365);
  
  drivetrain_.SetTarget(drivetrain_target);

}

void SpeakerAlignCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool SpeakerAlignCommand::IsFinished() {
  return is_done_;
}