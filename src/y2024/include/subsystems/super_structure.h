#ifndef Y2024_SUBSYSTEMS_SUPER_STRUCTURE_H_
#define Y2024_SUBSYSTEMS_SUPER_STRUCTURE_H_

#include "constants.h"
#include "frc846/control/motion.h"
#include "frc846/subsystem.h"
#include "frc846/util/pref.h"
#include "frc846/util/share_tables.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"

struct PWTSetpoint {
  units::degree_t pivot;
  units::inch_t telescope;
  units::degree_t wrist;
};

class PWTSetPref {
 public:
  PWTSetPref(std::string setpointName, PWTSetpoint backup)
      : pivot{frc846::motion::MotionTarget::preferences_loggable,
              "Pivot/" + setpointName, 0_deg},
        wrist{frc846::motion::MotionTarget::preferences_loggable,
              "Wrist/" + setpointName, 0_deg},
        telescope{frc846::motion::MotionTarget::preferences_loggable,
                  "Telescope/" + setpointName, 0_in} {}

  PWTSetpoint get() {
    return {pivot.value(), telescope.value(), wrist.value()};
  }

 private:
  frc846::Pref<units::degree_t> pivot;
  frc846::Pref<units::degree_t> wrist;
  frc846::Pref<units::inch_t> telescope;
};

struct SuperStructureReadings {};
struct SuperStructureTarget {};

class SuperStructureSubsystem
    : public frc846::Subsystem<SuperStructureReadings, SuperStructureTarget> {
 public:
  SuperStructureSubsystem(bool init);

  SuperStructureTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroSubsystem() { hasZeroed = true; }

  bool GetHasZeroed() { return hasZeroed; }

  frc846::Pref<double> manual_control_deadband_{
      *this, "manual_control_deadband", 0.05};

  frc846::Loggable auto_named_{*this, "auto"};
  frc846::Pref<units::second_t> post_shoot_wait_{auto_named_, "post_shoot_wait",
                                                 1_s};
  frc846::Pref<units::second_t> pre_shoot_wait_{auto_named_, "pre_shoot_wait",
                                                1_s};

  frc846::Loggable shooting_named_{*this, "shooting"};
  frc846::Pref<units::inch_t> teleop_shooter_height_{
      shooting_named_, "teleop_shooter_height", 39_in};
  frc846::Pref<units::inch_t> teleop_shooter_x_{shooting_named_,
                                                "teleop_shooter_x", 4_in};
  frc846::Pref<units::inch_t> auto_shooter_height_{
      shooting_named_, "auto_shooter_height", 21_in};

  frc846::Pref<units::inch_t> trap_start_x{shooting_named_, "trap_start_x",
                                           17_in};
  frc846::Pref<units::inch_t> trap_start_y{shooting_named_, "trap_start_y",
                                           25_in};
  frc846::Pref<units::inch_t> trap_end_x{shooting_named_, "trap_fin_x", 25_in};
  frc846::Pref<units::inch_t> trap_end_y{shooting_named_, "trap_fin_y", 47_in};
  frc846::Pref<units::degree_t> trap_start_angle{shooting_named_,
                                                 "trap_start_a", 60_deg};
  frc846::Pref<units::degree_t> trap_end_angle{shooting_named_, "trap_end_a",
                                               -10_deg};

  // SETPOINTS
  PWTSetPref amp_position_{"amp_position", {0_deg, 0_in, 0_deg}};
  PWTSetpoint getAmpSetpoint() { return amp_position_.get(); }

  PWTSetPref intake_position_{"intake_position", {0_deg, 0_in, 0_deg}};
  PWTSetpoint getIntakeSetpoint() { return intake_position_.get(); }

  PWTSetPref source_position_{"source_position", {0_deg, 0_in, 0_deg}};
  PWTSetpoint getSourceSetpoint() { return source_position_.get(); }

  PWTSetPref stow_position_{"stow_position", {0_deg, 0_in, 0_deg}};
  PWTSetpoint getStowSetpoint() { return stow_position_.get(); }

  PWTSetPref shoot_position_{"shoot_position", {0_deg, 0_in, 0_deg}};
  PWTSetpoint getShootSetpoint() { return shoot_position_.get(); }

  PWTSetPref auto_shoot_position_{"auto_shoot_position", {0_deg, 0_in, 0_deg}};
  PWTSetpoint getAutoShootSetpoint() { return auto_shoot_position_.get(); }

  PWTSetPref preclimb_position_{"preclimb_position", {0_deg, 0_in, 0_deg}};
  PWTSetpoint getPreClimbSetpoint() { return preclimb_position_.get(); }

 private:
  bool hasZeroed = false;

  SuperStructureReadings GetNewReadings() override;

  void DirectWrite(SuperStructureTarget target) override;
};

#endif