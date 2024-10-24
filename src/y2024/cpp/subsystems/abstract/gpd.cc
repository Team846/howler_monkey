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
    const frc846::util::Vector2D<units::feet_per_second_t>& robot_velocity) {
  if (notes.empty()) {
    return {};
  }
  frc846::util::Vector2D<units::foot_t> closest_note;
  int closest_note_index = -1;
  double max_dot_product = -std::numeric_limits<double>::infinity();
  std::pair<double, double> robot_unit_vector{
      robot_velocity.x.to<double>() / robot_velocity.Magnitude().to<double>(),
      robot_velocity.y.to<double>() / robot_velocity.Magnitude().to<double>()};
  for (int i = 0; i < notes.size(); i++) {  // find closest matching vector in
                                            // terms of angle to robot velocity
    frc846::util::Vector2D<units::foot_t> relative_note = notes.at(i);
    std::pair<double, double> note_unit_vector{
        relative_note.x.to<double>() / relative_note.Magnitude().to<double>(),
        relative_note.y.to<double>() / relative_note.Magnitude().to<double>()};
    double dot_product = robot_unit_vector.first * note_unit_vector.first +
                         robot_unit_vector.second * note_unit_vector.second;
    if (dot_product > max_dot_product) {
      std::cout << "x: " << relative_note.x.to<double>()
                << "y: " << relative_note.y.to<double>() << std::endl;
      max_dot_product = dot_product;
      closest_note = relative_note;
      closest_note_index = i;
    }
  }
  //   std::cout<<"x"<<closest_note.x.to<double>()<<"y"<<closest_note.y.to<double>()<<std::endl;
  return std::pair<frc846::util::Vector2D<units::foot_t>, int>{
      closest_note, closest_note_index};
}

frc846::util::Vector2D<units::foot_t> GPDSubsystem::findDistance(
    units::degree_t theta_h, units::degree_t theta_v) {
  units::foot_t height = mount_height_.value() - note_height_.value();
  auto dist = units::math::abs(
      height * units::math::tan(theta_v));  // absolute as saftey check
  auto yDist = dist * units::math::cos(theta_h);
  auto xDist = dist * units::math::sin(theta_h);

  return frc846::util::Vector2D<units::foot_t>{xDist, yDist};
}

GPDReadings GPDSubsystem::ReadFromHardware() {
  GPDReadings newReadings{};

  if (!requested) {
    prevFrame = (prevFrame + 1) % 500;
    gpdTable->PutNumber("frameRequested", prevFrame);
    nt_table.Flush();
    units::degree_t bearing_ =
        units::degree_t(frc846::util::ShareTables::GetDouble("robot_bearing_"));
    units::inch_t robot_x =
        units::foot_t(frc846::util::ShareTables::GetDouble("odometry_x_"));
    units::inch_t robot_y =
        units::foot_t(frc846::util::ShareTables::GetDouble("odometry_y_"));
    poseAtLastRequest = {{robot_x, robot_y}, bearing_};
    requested = true;
  }
  if (requested && gpdTable->GetNumber("frameSent", -2) == prevFrame) {
    std::vector<double> theta_h = gpdTable->GetNumberArray("theta_h", {});
    std::vector<double> theta_v = gpdTable->GetNumberArray("theta_v", {});
    //   std::cout<<bearing_.to<double>()<<std::endl;
    std::vector<frc846::util::Vector2D<units::foot_t>>
        points;  // shifted to robot pose, but oriented according to field
    for (int i = 0; i < theta_h.size(); i++) {
      frc846::util::Vector2D<units::foot_t> values =
          findDistance(theta_h[i] * 1_deg, theta_v[i] * 1_deg);
      values = values.Rotate(poseAtLastRequest.bearing);
      // std
      points.push_back(values);
    }
    //   std::cout<<points.size()<<"awpighwaugioh"<<std::endl;
    // points.push_back({1_ft, 1_ft});
    if (points.size() > 0) {
      std::pair<frc846::util::Vector2D<units::foot_t>, int> best_note =
          std::make_pair(points.at(0), 0);

      newReadings.note_detected = true;
      newReadings.points = points;
      newReadings.closest_note = best_note.first + poseAtLastRequest.point;
      newReadings.note_index = best_note.second;
      newReadings.note_angle = std::get<0>(best_note).Bearing();
    } else {
      newReadings.note_detected = false;
    }
    requested = false;
  } else {
    newReadings = prevReadings;
  }

  note_angle_graph_.Graph(newReadings.note_angle);
  note_distance_magnitude_graph_.Graph(newReadings.closest_note.Magnitude());
  frc846::util::ShareTables::SetDouble("closest_note_x",
                                       newReadings.closest_note.x.to<double>());
  frc846::util::ShareTables::SetDouble("closest_note_y",
                                       newReadings.closest_note.y.to<double>());

  prevReadings = newReadings;
  return newReadings;
  // if new frame is equal to current frame

  // DrivetrainReadings drivetrainReadings;

  // //   std::cout<<newReadings.note_angle.to<double>()<<std::endl;

  // return newReadings;
}

void GPDSubsystem::WriteToHardware(GPDTarget target) {}
