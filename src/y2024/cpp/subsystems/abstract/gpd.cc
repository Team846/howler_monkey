#include "subsystems/abstract/gpd.h"

#include <frc/DriverStation.h>
#include <units/math.h>

#include <vector>

#include "constants.h"
#include "field.h"
#include "frc846/ntinf/pref.h"
#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"

GPDSubsystem::GPDSubsystem(bool init)
    : frc846::robot::GenericSubsystem<GPDReadings, GPDTarget>("gpd", init) {
  if (init) {
  }
}

GPDTarget GPDSubsystem::ZeroTarget() const {
  GPDTarget target;
  return target;
}

bool GPDSubsystem::VerifyHardware() { return true; }

std::pair<frc846::util::Vector2D<units::foot_t>, int> GPDSubsystem::getBestNote(
    const std::vector<frc846::util::Vector2D<units::foot_t>>& notes,
    const frc846::util::Vector2D<units::foot_t>& robot_position,
    const frc846::util::Vector2D<units::feet_per_second_t>& robot_velocity) {
  if (!notes.empty()) {
    return {};
  }
  frc846::util::Vector2D<units::foot_t> closest_note;
  int closest_note_index = -1;
  double max_dot_product = -std::numeric_limits<double>::infinity();
  std::pair<double, double> robot_unit_vector{
      robot_velocity.x.to<double>() / robot_velocity.Magnitude().to<double>(),
      robot_velocity.y.to<double>() / robot_velocity.Magnitude().to<double>()};
  for (const auto& note : notes) {
    frc846::util::Vector2D<units::foot_t> relative_note = note - robot_position;
    std::pair<double, double> note_unit_vector{
        relative_note.x.to<double>() / relative_note.Magnitude().to<double>(),
        relative_note.y.to<double>() / relative_note.Magnitude().to<double>()};
    double dot_product = robot_unit_vector.first * note_unit_vector.first +
                         robot_unit_vector.second * note_unit_vector.second;
    if (dot_product > max_dot_product) {
      max_dot_product = dot_product;
      closest_note = note;
      closest_note_index = &note - &notes[0];
    }
  }
  return std::pair<frc846::util::Vector2D<units::foot_t>, int>{
      closest_note, closest_note_index};
}

frc846::util::Vector2D<units::foot_t> GPDSubsystem::findDistance(
    units::degree_t theta_h, units::degree_t theta_v) {
  units::foot_t mount_height = 2.0_ft;
  units::foot_t note_height = 0.0_ft;
  units::foot_t height = note_height - mount_height;
  auto yDist = height / units::math::tan(theta_v) * units::math::cos(theta_h);
  auto xDist = yDist * units::math::tan(theta_h) * units::math::sin(theta_h);

  return frc846::util::Vector2D<units::foot_t>{xDist, yDist};
}

GPDReadings GPDSubsystem::ReadFromHardware() {
  GPDReadings newReadings{};
  DrivetrainReadings drivetrainReadings;

  std::vector<double> note_x = gpdTable->GetNumberArray("x", {});
  std::vector<double> note_y = gpdTable->GetNumberArray("y", {});
  std::vector<double> theta_h = gpdTable->GetNumberArray("theta_h", {});
  std::vector<double> theta_v = gpdTable->GetNumberArray("theta_v", {});
  double camera_latency = gpdTable->GetNumber("latency", 0.0);

  std::vector<frc846::util::Vector2D<units::foot_t>> points;
  for (int i = 0; i < theta_h.size(); i++) {
    frc846::util::Vector2D<units::foot_t> values =
        findDistance(theta_h[i] * 1_deg, theta_v[i] * 1_deg);
    points.push_back(values);
  }

  std::pair<frc846::util::Vector2D<units::foot_t>, int> best_note = getBestNote(
      points, drivetrainReadings.pose.point, drivetrainReadings.velocity);

  newReadings.note_detected = note_x.size() != 0;
  newReadings.points = points;
  newReadings.closest_note = best_note.first;
  newReadings.note_index = best_note.second;
  newReadings.note_angle = units::math::atan2(newReadings.closest_note.y,
                                              newReadings.closest_note.x);

  return newReadings;
}

void GPDSubsystem::WriteToHardware(GPDTarget target) {}
