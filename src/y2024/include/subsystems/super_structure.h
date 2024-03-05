#ifndef Y2024_SUBSYSTEMS_SUPER_STRUCTURE_H_
#define Y2024_SUBSYSTEMS_SUPER_STRUCTURE_H_

#include "frc846/util/share_tables.h"
#include "frc846/subsystem.h"
#include "frc846/util/pref.h"
#include "units/length.h"
#include "units/time.h"
#include "units/angle.h"

struct SuperStructureReadings {};
struct SuperStructureTarget {};

class SuperStructureSubsystem : public frc846::Subsystem<SuperStructureReadings, SuperStructureTarget> {
 public:
  SuperStructureSubsystem(bool init);

  SuperStructureTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroSubsystem() {
    hasZeroed = true;
  }

  bool GetHasZeroed() { return hasZeroed; }

  frc846::Loggable auto_named_{*this, "auto"};
  frc846::Pref<units::second_t> post_shoot_wait_{auto_named_, "post_shoot_wait", 1_s};
  frc846::Pref<units::second_t> pre_shoot_wait_{auto_named_, "pre_shoot_wait", 1_s};

  frc846::Loggable shooting_named_{*this, "shooting"};
  frc846::Pref<units::inch_t> teleop_shooter_height_{shooting_named_, "teleop_shooter_height", 39_in};
  frc846::Pref<units::inch_t> teleop_shooter_x_{shooting_named_, "teleop_shooter_x", 4_in};
  frc846::Pref<units::foot_t> shooter_range_{shooting_named_, "shooter_range", 11_ft};
  frc846::Pref<double> shooter_speed_tolerance_{shooting_named_, "shooter_speed_tolerance", 0.2};
  frc846::Pref<double> shoot_angle_calc_tolerance_{shooting_named_, "shoot_angle_calc_tolerance", 0.06};
  frc846::Pref<double> shoot_angle_calc_max_iterations_{shooting_named_, "shoot_angle_calc_max_iterations", 5000};
  frc846::Pref<units::degree_t> shoot_angle_calc_intial_guess_{shooting_named_, "shoot_angle_calc_intial_guess", 1.01_deg};
  frc846::Pref<double> shoot_drive_angle_calc_tolerance_{shooting_named_, "shoot_drive_angle_calc_tolerance", 0.06};
  frc846::Pref<double> shoot_drive_angle_calc_max_iterations_{shooting_named_, "shoot_drive_angle_calc_max_iterations", 5000};
  frc846::Pref<units::degree_t> shoot_drive_angle_calc_intial_guess_{shooting_named_, "shoot_drive_angle_calc_intial_guess", 1.01_deg};

 private:
  
  bool hasZeroed = false;

  frc846::Pref<bool> is_red_side_{*this, "is_red_side", true};

  SuperStructureReadings GetNewReadings() override;

  void DirectWrite(SuperStructureTarget target) override;
};

#endif