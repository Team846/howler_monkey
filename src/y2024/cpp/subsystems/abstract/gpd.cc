#include "subsystems/abstract/gpd.h"

#include <frc/DriverStation.h>
#include <units/math.h>

#include "field.h"
#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/util/pref.h"
#include "constants.h"
#include <vector>

GPDSubsystem::GPDSubsystem(bool init)
    : frc846::Subsystem<GPDReadings, GPDTarget>("gpd", init) {
  if (init) {
  }
     
}

GPDTarget GPDSubsystem::ZeroTarget() const {
  GPDTarget target;
  return target;
}

bool GPDSubsystem::VerifyHardware() { return true; }

frc846::util::Vector2D<units::foot_t> GPDSubsystem::getBestNote(
    const std::vector<frc846::util::Vector2D<units::foot_t>>& notes,
    const frc846::util::Vector2D<units::foot_t>& robot_position,
    const frc846::util::Vector2D<units::feet_per_second_t>& robot_velocity) {
    if (!notes.empty()) { return {}; }
    frc846::util::Vector2D<units::foot_t> closest_note;
    double max_dot_product = -std::numeric_limits<double>::infinity();
    frc846::util::Vector2D<double> robot_unit_vector{
        robot_velocity.x.to<double>() / robot_velocity.Magnitude().to<double>(),
        robot_velocity.y.to<double>() / robot_velocity.Magnitude().to<double>()
    };
    for (const auto& note : notes) {
        frc846::util::Vector2D<units::foot_t> relative_note = note - robot_position;
        frc846::util::Vector2D<double> note_unit_vector{
            relative_note.x.to<double>() / relative_note.Magnitude().to<double>(),
            relative_note.y.to<double>() / relative_note.Magnitude().to<double>()
        };
        double dot_product = robot_unit_vector.x * note_unit_vector.x +
                             robot_unit_vector.y * note_unit_vector.y;
        if (dot_product > max_dot_product) {
            max_dot_product = dot_product;
            closest_note = note;
        }
    } return closest_note;
}


frc846::util::Vector2D<units::foot_t> findDistance(auto note_x, auto note_y) {
    auto mount_height = 1_ft * 2.5;
    auto note_height = 1_ft * 1/12;
    auto height = 1_ft * (mount_height - note_height);
    auto mount_angle = radians(0);

    auto horizontal_fov = radians(72.4);
    auto verticle_fov = radians(44.7);

    auto theta_h = horizontal_fov * (note_x[0]-0.5*256) / 256;
    auto theta_v = radians(90) - (verticle_fov * (note_y[0] - 0.5*256) / 256 + mount_angle);

    auto d = height * units::math::atan(theta_v);
    auto xDist = d * units::math::sin(theta_h);
    auto yDist = d * units::math::cos(theta_h);

    return frc846::util::Vector2D<units::foot_t>{
      xDist,
      yDist
    };
}

GPDReadings GPDSubsystem::GetNewReadings() { 
    GPDReadings newReadings{};
    DrivetrainReadings drivetrainReadings;

    auto note_x = gpdTable->GetNumberArray("x", {});
    auto note_y = gpdTable->GetNumberArray("y", {});
    double camera_latency = gpdTable->GetNumber("latency", 0.0);

    std::vector<frc846::util::Vector2D<units::foot_t>> points;
    for (int i = 0; i < note_x.size(); i++) {
      frc846::util::Vector2D<units::foot_t> values = findDistance(note_x[i], note_y[i]);
      points.push_back(values);
    }

    newReadings.note_detected = note_x.size() != 0;
    newReadings.points = points;
    newReadings.closest_note = getBestNote(points, drivetrainReadings.pose.point, drivetrainReadings.velocity);

    return newReadings;
}

void GPDSubsystem::DirectWrite(GPDTarget target) {}

