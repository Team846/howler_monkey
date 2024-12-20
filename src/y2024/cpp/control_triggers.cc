#include "control_triggers.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>

#include "commands/basic/amp_command.h"
#include "commands/basic/deploy_intake_command.h"
#include "commands/basic/eject_command.h"
#include "commands/basic/pass_command.h"
#include "commands/basic/prepare_shoot_command.h"
#include "commands/basic/pull_command.h"
#include "commands/basic/shoot_command.h"
#include "commands/basic/source_command.h"
#include "commands/basic/spin_up_command.h"
#include "commands/basic/trap_command.h"
#include "commands/basic/wrist_zero_command.h"

void ControlTriggerInitializer::InitTeleopTriggers(RobotContainer& container) {
  frc2::Trigger drivetrain_zero_bearing_trigger{
      [&] { return container.control_input_.GetReadings().zero_bearing; }};

  drivetrain_zero_bearing_trigger.WhileTrue(
      frc2::InstantCommand([&] {
        container.drivetrain_.SetBearing(
            frc846::util::ShareTables::GetBoolean("is_red_side") ? 0_deg
                                                                 : 180_deg);
      }).ToPtr());

  frc2::Trigger on_piece_trigger{
      [&] { return container.intake_.GetHasPiece(); }};

  on_piece_trigger.OnTrue(
      frc2::InstantCommand(
          [&] { container.control_input_.SetTarget({true, false}); })
          .WithTimeout(1_s)
          .AndThen(frc2::WaitCommand(1_s).ToPtr())
          .AndThen(frc2::InstantCommand([&] {
                     container.control_input_.SetTarget({false, false});
                   }).ToPtr()));

  frc2::Trigger shooter_spinup_trigger{[&] {
    return container.shooter_.GetReadings().error_percent <=
           container.shooter_.shooter_speed_tolerance_.value();
  }};

  shooter_spinup_trigger.OnTrue(
      frc2::InstantCommand(
          [&] { container.control_input_.SetTarget({true, false}); })
          .WithTimeout(1_s)
          .AndThen(frc2::WaitCommand(1_s).ToPtr())
          .AndThen(frc2::InstantCommand([&] {
                     container.control_input_.SetTarget({false, false});
                   }).ToPtr()));

  frc2::Trigger amp_command_trigger{[&container] {
    return container.control_input_.GetReadings().running_amp;
  }};

  amp_command_trigger.WhileTrue(AmpCommand{container}.ToPtr());

  // frc2::Trigger super_amp_trigger_red{[&container] {
  //   return container.control_input_.GetReadings().running_super_amp &&
  //          frc846::util::ShareTables::GetBoolean("is_red_side");
  // }};
  // frc2::Trigger super_amp_trigger_blue{[&container] {
  //   return container.control_input_.GetReadings().running_super_amp &&
  //          !frc846::util::ShareTables::GetBoolean("is_red_side");
  // }};

  // super_amp_trigger_red.WhileTrue(SuperAmpCommand{container, true}.ToPtr());
  // super_amp_trigger_blue.WhileTrue(SuperAmpCommand{container,
  // false}.ToPtr());

  frc2::Trigger intake_command_trigger{[&container] {
    return container.control_input_.GetReadings().running_intake;
  }};

  intake_command_trigger.WhileTrue(DeployIntakeCommand{container}.ToPtr());

  frc2::Trigger source_command_trigger{[&container] {
    return container.control_input_.GetReadings().running_source;
  }};

  source_command_trigger.WhileTrue(SourceCommand{container}.ToPtr());

  frc2::Trigger point_blank_prep_speaker_trigger{[&container] {
    return container.control_input_.GetReadings().running_prep_shoot;
  }};

  point_blank_prep_speaker_trigger.WhileTrue(
      PrepareShootCommand{container, false}.ToPtr());

  frc2::Trigger pass_trigger{[&container] {
    return container.control_input_.GetReadings().running_pass;
  }};

  pass_trigger.WhileTrue(PassCommand{container}.ToPtr());

  point_blank_prep_speaker_trigger.WhileTrue(
      PrepareShootCommand{container, false}.ToPtr());

  frc2::Trigger prep_super_shot_trigger{[&container] {
    return container.control_input_.GetReadings().running_super_shoot;
  }};

  prep_super_shot_trigger.WhileTrue(
      PrepareShootCommand{container, true}.ToPtr());

  frc2::Trigger operator_pull_trigger{[&container] {
    return container.control_input_.GetReadings().manual_feed;
  }};
  operator_pull_trigger.WhileTrue(PullCommand{container}.ToPtr());

  frc2::Trigger operator_spin_trigger{[&container] {
    return container.control_input_.GetReadings().manual_spin_up;
  }};
  operator_spin_trigger.WhileTrue(SpinUpCommand{container}.ToPtr());

  frc2::Trigger eject_trigger{
      [&container] { return container.control_input_.GetReadings().eject; }};

  eject_trigger.WhileTrue(EjectCommand{container}.ToPtr());

  frc2::Trigger shoot_trigger{
      [&container] { return container.control_input_.GetReadings().shooting; }};

  shoot_trigger.OnTrue(ShootCommand{container}.ToPtr());

  for (int i = 0; i <= 6; i++) {
    frc2::Trigger trap_stage_trigger{[&container, i] {
      return container.control_input_.GetReadings().stageOfTrap == i;
    }};
    trap_stage_trigger.WhileTrue(TrapCommand{container, i}.ToPtr());
  }

  frc2::Trigger wrist_zero_trigger{
      [&] { return container.control_input_.GetReadings().home_wrist; }};

  wrist_zero_trigger.OnTrue(WristZeroCommand{container}.ToPtr());

  frc2::Trigger wrist_trim_up_trigger{
      [&] { return container.control_input_.GetReadings().trim_up; }};

  wrist_trim_up_trigger.OnTrue(frc2::InstantCommand([&] {
                                 container.super_structure_.TrimUp();
                               }).ToPtr());

  frc2::Trigger wrist_trim_down_trigger{
      [&] { return container.control_input_.GetReadings().trim_down; }};

  wrist_trim_down_trigger.OnTrue(frc2::InstantCommand([&] {
                                   container.super_structure_.TrimDown();
                                 }).ToPtr());
}