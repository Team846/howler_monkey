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

struct VisionReadings {
  units::foot_t est_dist_from_speaker_x;
  units::foot_t est_dist_from_speaker_y;
  units::foot_t est_dist_from_speaker;
  units::inch_t local_x_dist;
  units::inch_t local_y_dist;
  units::inch_t tag_distance;
  units::degree_t tag_angle_difference;

  double velocity_in_component;
  double velocity_orth_component;
};

struct VisionTarget {};

struct AprilTagData {
  units::inch_t x_pos;
  units::inch_t y_pos;
  units::degree_t angle;
  units::inch_t height;
};

class VisionSubsystem : public frc846::Subsystem<VisionReadings, VisionTarget> {
 public:
  VisionSubsystem(bool init);

  void Setup() override {};

  VisionTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::Pref<bool> default_is_red_side_{*this, "default_is_red_side"};

  frc846::Grapher<bool> tag_in_sight_graph_{*this, "tag_in_sight"};
  frc846::Grapher<int> tag_id_graph_{*this, "tag_id"};
  frc846::Grapher<units::millisecond_t> ll_latency_graph_{*this, "ll_latency"};

  frc846::Loggable readings_named{*this, "readings"};
  frc846::Grapher<units::foot_t> speaker_x_dist_graph_{readings_named,
                                                       "speaker_dist_x"};
  frc846::Grapher<units::foot_t> speaker_y_dist_graph_{readings_named,
                                                       "speaker_dist_y"};
  frc846::Grapher<units::foot_t> speaker_dist_graph_{readings_named,
                                                     "speaker_dist_x"};
  frc846::Grapher<units::foot_t> local_x_dist_graph_{readings_named,
                                                     "local_dist_x"};
  frc846::Grapher<units::foot_t> local_y_dist_graph_{readings_named,
                                                     "local_dist_y"};
  frc846::Grapher<units::foot_t> tag_distance_graph_{readings_named,
                                                     "tag_distance"};
  frc846::Grapher<units::degree_t> tag_angle_difference_graph_{
      readings_named, "tag_angle_difference"};

  frc846::Grapher<double> velocity_in_component_graph_{readings_named,
                                                       "velocity_in_component"};
  frc846::Grapher<double> velocity_orth_component_graph_{
      readings_named, "velocity_orth_component"};

  std::shared_ptr<nt::NetworkTable> table =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  frc846::Pref<units::millisecond_t> can_bus_latency_{*this, "can_bus_latency",
                                                      20_ms};

  // SPEAKER = Blue 7, Red 4
  std::map<int, AprilTagData> tag_locations{
      {7, {-1.5_in, 218.42_in, 0_deg, 57.13_in}},
      {8, {-1.5_in, 196.17_in, 0_deg, 57.13_in}},
      {4, {652.73_in, 218.4_in, 180_deg, 57.13_in}},
      {3, {652.73_in, 196.17_in, 180_deg, 57.13_in}}};

  frc846::Pref<units::inch_t> camera_height_{*this, "camera_height", 14_in};
  frc846::Pref<units::degree_t> camera_angle_{*this, "camera_angle", 37_deg};
  frc846::Pref<units::inch_t> camera_y_offset_{*this, "camera_y_offset_",
                                               -10_in};
  frc846::Pref<units::inch_t> camera_x_offset_{*this, "camera_x_offset_", 7_in};

  units::inch_t x_correction = 0_in;
  units::inch_t y_correction = 0_in;

  VisionReadings GetNewReadings() override;

  void DirectWrite(VisionTarget target) override;
};
