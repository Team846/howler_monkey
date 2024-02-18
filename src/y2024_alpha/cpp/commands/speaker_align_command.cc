#include "commands/speaker_align_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

#include "subsystems/field.h"

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
  auto target_x = field::points::kSpeaker().x;
  auto target_y = field::points::kSpeaker().y;

  auto dist_x = target_x - current_x;
  auto dist_y = target_y - current_y;

  auto drivetrain_target = drivetrain_.ZeroTarget();

  units::degree_t target_angle = units::math::atan2(dist_x, dist_y);
  
  drivetrain_target.rotation = DrivetrainRotationPosition(target_angle + 180_deg);

  auto detect_angle = std::abs(360 - ((int)(units::math::abs((target_angle + 180_deg)).to<double>() 
    - drivetrain_.readings().pose.bearing.to<double>())));

  is_done_ = detect_angle < 5 || (detect_angle > 355 && detect_angle < 365);
  
  drivetrain_.SetTarget(drivetrain_target);

}

void SpeakerAlignCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool SpeakerAlignCommand::IsFinished() {
  return is_done_;
}