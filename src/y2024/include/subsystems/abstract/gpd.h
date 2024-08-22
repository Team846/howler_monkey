#pragma once

#include <units/angle.h>
#include <units/length.h>

#include "frc/smartdashboard/Smartdashboard.h"
#include "frc846/subsystem.h"
#include "frc846/util/grapher.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "ports.h"

struct GPDTarget {};
struct GPDReadings {
  units::degree_t note_angle;
  bool note_detected;
  units::foot_t note_distance;
  units::foot_t note_x;
  units::foot_t note_y;
};

class GPDSubsystem : public frc846::Subsystem<GPDReadings, GPDTarget> {
 public:
  GPDSubsystem(bool init);

  void Setup() override {};

  GPDTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
    frc846::Loggable readings_named{*this, "readings"};
    frc846::Grapher<bool> note_detected_graph_{
        readings_named, "note_detected"};

    std::shared_ptr<nt::NetworkTable> gpdTable = nt::NetworkTableInstance::GetDefault().GetTable("gpd");
    frc846::Grapher<units::foot_t> note_x_distance_graph_{readings_named, "note_x_distance_graph"};
    frc846::Grapher<units::foot_t> note_y_distance_graph_{readings_named, "note_y_distance_graph"};
    frc846::Grapher<units::second_t> note_camera_latency_{readings_named, "note_camera_latency_graph"};

    frc846::Grapher<units::angle::degree_t> note_angle_graph_{
        readings_named, "note_angle"};

    frc846::Grapher<units::length::foot_t> note_distance_graph_{
        readings_named, "note_distance"};

    GPDReadings GetNewReadings() override;

    void DirectWrite(GPDTarget target) override;
};
