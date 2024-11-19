#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/length.h>

#include "frc/smartdashboard/Smartdashboard.h"
#include "frc846/math/vectors.h"
#include "frc846/ntinf/grapher.h"
#include "frc846/robot/GenericSubsystem.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "ports.h"
#include "subsystems/hardware/drivetrain.h"

struct GPDTarget {};
struct GPDReadings {
  std::vector<frc846::math::Vector2D> notes;
};

class GPDSubsystem
    : public frc846::robot::GenericSubsystem<GPDReadings, GPDTarget> {
 public:
  GPDSubsystem(bool init);

  void Setup() override {};

  frc846::math::Vector2D findDistance(units::degree_t theta_h,
                                      units::degree_t theta_v);

  GPDTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc::Field2d g_field;

  GPDReadings prevReadings;

  frc846::base::Loggable readings_named{*this, "readings"};

  frc846::base::Loggable algo_params{*this, "algo_parameters"};
  frc846::ntinf::Pref<units::foot_t> mount_height_{algo_params, "mount_height",
                                                   1_ft};
  frc846::ntinf::Pref<units::foot_t> note_height_{algo_params, "note_height",
                                                  0_ft};
  frc846::ntinf::Pref<units::second_t> nt_latency{algo_params, "nt_latency",
                                                  0_s};

  std::shared_ptr<nt::NetworkTable> gpdTable =
      nt::NetworkTableInstance::GetDefault().GetTable("gpd");

  GPDReadings ReadFromHardware() override;

  void WriteToHardware(GPDTarget target) override;
};