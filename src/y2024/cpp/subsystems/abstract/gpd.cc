using namespace std;
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


std::vector<units::foot_t> findDistance(auto note_x, auto note_y) {
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

    return vector<units::foot_t>{
      xDist, yDist
    }
}

GPDReadings GPDSubsystem::GetNewReadings() { 
    GPDReadings newReadings{};

    auto note_x = gpdTable->GetNumberArray("x", {});
    auto note_y = gpdTable->GetNumberArray("y", {});
    double camera_latency = gpdTable->GetNumber("latency", 0.0);

    vector<vector<units::foot_t>> points;
    vector<units::degree_t> note_angle;
    for (int i = 0; i < note_x.size(); i++) {
      vector<units::foot_t> values = findDistance(note_x[i], note_y[i]);
      points[i] = values;

      // from vertical
      note_angle[i] = units::math::atan2(values[0], values[1])-90_deg;
    }

    newReadings.note_detected = note_x.size() != 0;
    newReadings.points = points;
    newReadings.note_angle = note_angle;

    return newReadings;


}
void GPDSubsystem::DirectWrite(GPDTarget target) {}

