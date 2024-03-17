#ifndef y2024_COMMANDS_SPEAKER_ALIGN_COMMAND_H_
#define y2024_COMMANDS_SPEAKER_ALIGN_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/scorer.h"
#include "subsystems/pivot.h"
#include "subsystems/telescope.h"
#include "subsystems/wrist.h"
#include "subsystems/robot_container.h"
#include "frc846/other/trajectory_generator.h"

class SpeakerAlignCommand
    : public frc2::CommandHelper<frc2::Command, SpeakerAlignCommand>,
      public frc846::Loggable {
 public:
  SpeakerAlignCommand(RobotContainer& container, frc846::InputWaypoint shoot_point);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DrivetrainSubsystem& drivetrain_;

  frc846::InputWaypoint shoot_point_;

  bool is_done_ = false;
};

#endif  // y2024_COMMANDS_SPEAKER_ALIGN_COMMAND_H_