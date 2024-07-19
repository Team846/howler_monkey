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
#include "commands/basic/trap_command.h"
#include "commands/basic/wrist_zero_command.h"

void ControlTriggerInitializer::InitTeleopTriggers(RobotContainer& container) {
  frc2::Trigger amp_command_trigger{
      [&container] { return container.control_input_.readings().running_amp; }};

  amp_command_trigger.OnTrue(AmpCommand{container}.ToPtr());
  // amp_command_trigger.OnFalse(StowCommand{container}.ToPtr());

  frc2::Trigger intake_command_trigger{[&container] {
    return container.control_input_.readings().running_intake;
  }};

  intake_command_trigger.OnTrue(DeployIntakeCommand{container}.ToPtr());
  // intake_command_trigger.OnFalse(StowCommand{container}.ToPtr());

  frc2::Trigger source_command_trigger{[&container] {
    return container.control_input_.readings().running_source;
  }};

  source_command_trigger.OnTrue(SourceCommand{container}.ToPtr());
  // source_command_trigger.OnFalse(StowCommand{container}.ToPtr());

  frc2::Trigger point_blank_prep_speaker_trigger{[&container] {
    return container.control_input_.readings().running_prep_shoot;
  }};

  point_blank_prep_speaker_trigger.OnTrue(
      PrepareShootCommand{container, false}.ToPtr());
  // point_blank_prep_speaker_trigger.OnFalse(StowCommand{container}.ToPtr());

  frc2::Trigger prep_super_shot_trigger{[&container] {
    return container.control_input_.readings().running_super_shoot;
  }};

  prep_super_shot_trigger.OnTrue(PrepareShootCommand{container, true}.ToPtr());
  // prep_super_shot_trigger.OnFalse(StowCommand{container}.ToPtr());

  frc2::Trigger operator_pull_trigger{
      [&container] { return container.control_input_.readings().manual_feed; }};
  operator_pull_trigger.OnTrue(PullCommand{container}.ToPtr());

  frc2::Trigger operator_spin_trigger{[&container] {
    return container.control_input_.readings().manual_spin_up;
  }};
  operator_spin_trigger.OnTrue(SpinUpCommand{container}.ToPtr());

  frc2::Trigger eject_trigger{
      [&container] { return container.control_input_.readings().eject; }};

  eject_trigger.OnTrue(EjectCommand{container}.ToPtr());

  frc2::Trigger shoot_trigger{
      [&container] { return container.control_input_.readings().shooting; }};

  shoot_trigger.OnTrue(ShootCommand{container}.ToPtr());

  frc2::Trigger idle_shooter{[&container] {
    return !container.control_input_.readings().shooting &&
           !container.control_input_.readings().running_prep_shoot &&
           !container.control_input_.readings().running_super_shoot &&
           !container.control_input_.readings().manual_spin_up &&
           !container.control_input_.readings().running_pass &&
           !container.control_input_.readings().manual_feed;
  }};

  idle_shooter.WhileTrue(IdleCommand{container, false, true}.ToPtr());

  frc2::Trigger idle_intake{[&container] {
    return !container.control_input_.readings().running_intake &&
           !container.control_input_.readings().running_source &&
           !container.control_input_.readings().running_pass &&
           !container.control_input_.readings().manual_feed &&
           !container.control_input_.readings().eject &&
           container.control_input_.readings().stageOfTrap == 0;
  }};

  idle_intake.WhileTrue(IdleCommand{container, true, false}.ToPtr());

  frc2::Trigger stow_trigger{[&container] {
    return !container.control_input_.readings().running_intake &&
           !container.control_input_.readings().running_source &&
           !container.control_input_.readings().running_pass &&
           !container.control_input_.readings().running_amp &&
           !container.control_input_.readings().running_prep_shoot &&
           !container.control_input_.readings().running_super_shoot &&
           container.control_input_.readings().stageOfTrap == 0;
  }};

  stow_trigger.WhileTrue(StowCommand{container}.ToPtr());

  for (int i = 0; i <= 6; i++) {
    frc2::Trigger trap_stage_trigger{[&container, i] {
      return container.control_input_.readings().stageOfTrap == i;
    }};
    trap_stage_trigger.OnTrue(TrapCommand{container, i}.Until([&container, i] {
      return container.control_input_.readings().stageOfTrap != i;
    }));
  }

  frc2::Trigger wrist_zero_trigger{
      [&container] { return container.operator_.readings().x_button; }};

  wrist_zero_trigger.OnTrue(WristZeroCommand{container}.ToPtr());
}