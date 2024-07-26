#include "subsystems/abstract/vision.h"

#include <frc/DriverStation.h>
#include <units/math.h>

#include "field.h"
#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"

VisionSubsystem::VisionSubsystem(bool init)
    : frc846::Subsystem<VisionReadings, VisionTarget>("vision", init) {
  if (init) {
  }
}

VisionTarget VisionSubsystem::ZeroTarget() const {
  VisionTarget target;
  return target;
}

bool VisionSubsystem::VerifyHardware() { return true; }

VisionReadings VisionSubsystem::GetNewReadings() {
  if (auto alliance = frc::DriverStation::GetAlliance()) {
    if (alliance.value() == frc::DriverStation::Alliance::kRed) {
      frc846::util::ShareTables::SetBoolean("is_red_side", true);
    }
    if (alliance.value() == frc::DriverStation::Alliance::kBlue) {
      frc846::util::ShareTables::SetBoolean("is_red_side", false);
    }
  } else {
    frc846::util::ShareTables::SetBoolean("is_red_side",
                                          default_is_red_side_.value());
  }

  VisionReadings newReadings{};

  double targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
  double tag_id = table->GetNumber("tid", 0.0);

  tag_id_graph_.Graph(tag_id);

  double latency = table->GetNumber("tl", 0.0) + table->GetNumber("cl", 0.0);

  ll_latency_graph_.Graph(latency * 1_s);

  units::degree_t bearing_ =
      units::degree_t(frc846::util::ShareTables::GetDouble("robot_bearing_"));

  units::inch_t robot_x =
      units::foot_t(frc846::util::ShareTables::GetDouble("odometry_x_"));

  units::inch_t robot_y =
      units::foot_t(frc846::util::ShareTables::GetDouble("odometry_y_"));

  units::feet_per_second_t velocity_x = units::feet_per_second_t(
      frc846::util::ShareTables::GetDouble("velocity_x_"));

  units::feet_per_second_t velocity_y = units::feet_per_second_t(
      frc846::util::ShareTables::GetDouble("velocity_y_"));

  if (tag_locations.contains(tag_id)) {  // &&
    // frc846::util::ShareTables::GetString("mode").compare("kTeleop") == 0) {
    tag_in_sight_graph_.Graph(true);

    auto april_tag = tag_locations[tag_id];
    units::inch_t height_difference_ =
        april_tag.height - camera_height_.value();

    units::degree_t april_tag_vertical_angle =
        units::degree_t(targetOffsetAngle_Vertical) + camera_angle_.value();

    newReadings.tag_distance =
        height_difference_ / units::math::tan(april_tag_vertical_angle);

    newReadings.local_y_dist =
        newReadings.tag_distance *
            units::math::cos(units::degree_t(targetOffsetAngle_Horizontal) +
                             180_deg) +
        camera_y_offset_.value();
    newReadings.local_x_dist =
        newReadings.tag_distance *
            units::math::sin(units::degree_t(targetOffsetAngle_Horizontal) +
                             180_deg) +
        camera_x_offset_.value();

    robot_y += can_bus_latency_.value() * velocity_y;

    robot_x += can_bus_latency_.value() * velocity_x;

    // auto flipped = frc846::util::FieldPoint::flip(
    //     {{robot_x, robot_y}, bearing_},
    //     !frc846::util::ShareTables::GetBoolean("is_red_side"), true);

    // robot_y = flipped.point.y;
    // robot_x = flipped.point.x;
    // bearing_ = flipped.bearing;

    frc846::util::Vector2D<units::inch_t> local_tag_vec{
        newReadings.local_x_dist, newReadings.local_y_dist};
    auto tag_vec = local_tag_vec.Rotate(bearing_);

    units::inch_t tag_x_dist = tag_vec.x;

    units::inch_t tag_y_dist = tag_vec.y;

    y_correction =
        (april_tag.y_pos - tag_y_dist) - (robot_y - latency * 1_s * velocity_y);

    x_correction =
        (april_tag.x_pos - tag_x_dist) - (robot_x - latency * 1_s * velocity_x);
  } else {
    tag_in_sight_graph_.Graph(false);
  }

  robot_y += can_bus_latency_.value() * velocity_y + y_correction;

  robot_x += can_bus_latency_.value() * velocity_x + x_correction;

  newReadings.est_dist_from_speaker_x =
      robot_x - field::points::kSpeaker(
                    !frc846::util::ShareTables::GetBoolean("is_red_side"))
                    .x;

  newReadings.est_dist_from_speaker_y =
      robot_y - field::points::kSpeaker(
                    !frc846::util::ShareTables::GetBoolean("is_red_side"))
                    .y;

  auto point_target = frc846::util::Vector2D<units::foot_t>{
      -newReadings.est_dist_from_speaker_x,
      -newReadings.est_dist_from_speaker_y};

  newReadings.velocity_in_component =
      ((velocity_x * point_target.x + velocity_y * point_target.y) /
       point_target.Magnitude())
          .to<double>();

  point_target = point_target.Rotate(90_deg);

  newReadings.velocity_orth_component =
      ((velocity_x * point_target.x + velocity_y * point_target.y) /
       point_target.Magnitude())
          .to<double>();

  newReadings.est_dist_from_speaker = units::math::sqrt(
      units::math::pow<2, units::foot_t>(newReadings.est_dist_from_speaker_x) +
      units::math::pow<2, units::foot_t>(newReadings.est_dist_from_speaker_y));

  speaker_x_dist_graph_.Graph(newReadings.est_dist_from_speaker_x);
  speaker_y_dist_graph_.Graph(newReadings.est_dist_from_speaker_y);
  speaker_dist_graph_.Graph(newReadings.est_dist_from_speaker);
  local_x_dist_graph_.Graph(newReadings.local_x_dist);
  local_y_dist_graph_.Graph(newReadings.local_y_dist);
  tag_distance_graph_.Graph(newReadings.tag_distance);
  tag_angle_difference_graph_.Graph(newReadings.tag_angle_difference);
  velocity_in_component_graph_.Graph(newReadings.velocity_in_component);
  velocity_orth_component_graph_.Graph(newReadings.velocity_orth_component);

  return newReadings;
}

void VisionSubsystem::DirectWrite(VisionTarget target) {}
