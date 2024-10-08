#include "autos/GenericAuto.h"

#include <frc2/command/WaitCommand.h>

#include "autos/ActionMaker.h"

GenericAuto::GenericAuto(RobotContainer& container, AutoData data,
                         bool is_blue_side)
    : frc846::robot::GenericCommandGroup<RobotContainer, GenericAuto,
                                         frc2::SequentialCommandGroup>{
          container, data.name,
          frc2::SequentialCommandGroup{
              std::move(buildActionsGroup(data, container, is_blue_side))}} {}

std::vector<std::unique_ptr<frc2::Command>> GenericAuto::buildActionsGroup(
    AutoData data, RobotContainer& container, bool is_blue_side) {
  std::vector<std::unique_ptr<frc2::Command>> cmds{};
  cmds.push_back(std::make_unique<frc2::InstantCommand>([&, auto_data = data] {
    Log("Starting Auto: {}.", auto_data.name);

    int mirror = is_blue_side ? (int)auto_data.blue : (int)auto_data.red;

    auto start = mirror == 0 ? auto_data.start
                             : (mirror == 1 ? auto_data.start.mirror()
                                            : auto_data.start.mirrorOnlyY());

    container.drivetrain_.SetPoint(start.point);
    container.drivetrain_.SetBearing(start.bearing);
  }));
  for (auto& action : data.actions) {
    if (auto* action_name = std::get_if<std::string>(&action)) {
      cmds.push_back(ActionMaker::GetAction(*action_name, container));
    } else if (auto* fp = std::get_if<std::vector<frc846::math::FieldPoint>>(
                   &action)) {
      cmds.push_back(std::make_unique<frc846::swerve::FollowTrajectoryCommand>(
          container, *fp, is_blue_side ? (int)data.blue : (int)data.red));
    }
  }

  return cmds;
}