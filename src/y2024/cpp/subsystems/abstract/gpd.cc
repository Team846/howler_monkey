#include "subsystems/abstract/gpd.h"

#include <frc/DriverStation.h>
#include <units/math.h>

#include "field.h"
#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"

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


GPDReadings GPDSubsystem::GetNewReadings() { 
    GPDReadings newReadings{};

    auto note_x_theta = 1_ft* gpdTable->GetNumber("note_x", 0.0);
    auto note_y_theta = 1_ft* gpdTable->GetNumber("note_y", 0.0);
    auto note_angle = gpdTable->GetNumber("not_angle",0.0);
    auto note_distance = 1_ft*gpdTable->GetNumber("note_distance_magnitude",0.0);
    double camera_latency = gpdTable->GetNumber("latency", 0.0);
    
    //old code
    //newReadings.note_angle = units::math::atan2(note_x_theta, note_y_theta);
    //newReadings.note_distance = units::math::sqrt((note_x_theta * note_x_theta + note_y_theta * note_y_theta)); 
    
    newReadings.note_x = units::math::sin(note_angle) * note_distance;
    newReadings.note_y = units::math::cos(note_angle) * note_distance;

    note_detected_graph_.Graph(newReadings.note_detected);
    note_angle_graph_.Graph(newReadings.note_angle);
    note_distance_graph_.Graph(newReadings.note_distance);
    return newReadings;


}
void GPDSubsystem::DirectWrite(GPDTarget target) {}

