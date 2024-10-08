#include "autos/ActionMaker.h"

#include "commands/basic/auto_deploy_intake_command.h"
#include "commands/basic/auto_shoot_command.h"
#include "commands/basic/prepare_auto_shoot_command.h"
#include "commands/complex/home_during_auto_command.h"

std::unique_ptr<frc2::Command> ActionMaker::GetAction(
    std::string name, RobotContainer& container) {
  if (name == "shoot") {
    return std::make_unique<AutoShootCommand>(container);
  } else if (name == "prep_shoot") {
    return std::make_unique<PrepareAutoShootCommand>(container);
  } else if (name == "deploy_intake") {
    return std::make_unique<AutoDeployIntakeCommand>(container);
  } else if (name == "auto_home" || true) {
    return std::make_unique<HomeDuringAutoCommand>(container);
  }
  return nullptr;
}