#include "control_triggers.h"

#include <frc/DataLogManager.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>

#include "commands/basic/amp_command.h"
#include "commands/basic/deploy_intake_command.h"
#include "commands/basic/eject_command.h"
#include "commands/basic/idle_command.h"
#include "commands/basic/prepare_shoot_command.h"
#include "commands/basic/pull_command.h"
#include "commands/basic/shoot_command.h"
#include "commands/basic/source_command.h"
#include "commands/basic/spin_up_command.h"
#include "commands/basic/stow_command.h"

void ControlTriggerInitializer::InitTeleopTriggers(RobotContainer& container) {
  frc2::Trigger amp_command_trigger{
      [&container] { return container.control_input_.readings().running_amp; }};

  amp_command_trigger.OnTrue(AmpCommand{container}.ToPtr());
  amp_command_trigger.OnFalse(StowCommand{container}.ToPtr());

  frc2::Trigger intake_command_trigger{[&container] {
    return container.control_input_.readings().running_intake;
  }};

  intake_command_trigger.OnTrue(DeployIntakeCommand{container}.ToPtr());
  intake_command_trigger.OnFalse(StowCommand{container}.ToPtr());

  frc2::Trigger source_command_trigger{[&container] {
    return container.control_input_.readings().running_source;
  }};

  source_command_trigger.OnTrue(SourceCommand{container}.ToPtr());
  source_command_trigger.OnFalse(StowCommand{container}.ToPtr());

  frc2::Trigger point_blank_prep_speaker_trigger{[&container] {
    return container.control_input_.readings().running_prep_shoot;
  }};

  //point_blank_prep_speaker_trigger.OnTrue(
      //PrepareShootCommand{container, false}.ToPtr());
  point_blank_prep_speaker_trigger.OnFalse(StowCommand{container}.ToPtr());

  frc2::Trigger prep_super_shot_trigger{[&container] {
    return container.control_input_.readings().running_prep_shoot;
  }};

  //prep_super_shot_trigger.OnTrue(PrepareShootCommand{container, true}.ToPtr());
  prep_super_shot_trigger.OnFalse(StowCommand{container}.ToPtr());

  frc2::Trigger operator_pull_trigger{[&container] {
    return container.control_input_.readings().running_intake;
  }};
  operator_pull_trigger.OnTrue(PullCommand{container}.ToPtr());
  operator_pull_trigger.OnFalse(IdleCommand{container, false}.ToPtr());

  frc2::Trigger eject_trigger{
      [&container] { return container.control_input_.readings().eject; }};

  eject_trigger.OnTrue(EjectCommand{container}.ToPtr());
  eject_trigger.OnFalse(IdleCommand{container, false}.ToPtr());

  frc2::Trigger shoot_trigger{
      [&container] { return container.control_input_.readings().shooting; }};

  shoot_trigger.OnTrue(ShootCommand{container}.ToPtr());
  shoot_trigger.OnFalse(IdleCommand{container, false}.ToPtr());
}