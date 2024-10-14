#pragma once

#include "calculators/arm_kinematics_calculator.h"
#include "frc846/control/motion.h"
#include "frc846/ntinf/pref.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/util/share_tables.h"
#include "subsystems/hardware/pivot.h"
#include "subsystems/hardware/telescope.h"
#include "subsystems/hardware/wrist.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"

struct PTWSetpoint {
  units::degree_t pivot;
  units::inch_t telescope;
  units::degree_t wrist;

  PTWSetpoint operator+(const PTWSetpoint& other) const {
    return {other.pivot + this->pivot, other.telescope + this->telescope,
            other.wrist + this->wrist};
  }
};

class PTWSetPref {
 public:
  PTWSetPref(std::string setpointName, PTWSetpoint backup)
      : pivot{frc846::motion::MotionTarget::preferences_loggable,
              "Pivot/" + setpointName, 0_deg},
        wrist{frc846::motion::MotionTarget::preferences_loggable,
              "Wrist/" + setpointName, 0_deg},
        telescope{frc846::motion::MotionTarget::preferences_loggable,
                  "Telescope/" + setpointName, 0_in} {}

  PTWSetpoint get() {
    return {pivot.value(), telescope.value(), wrist.value()};
  }

 private:
  frc846::ntinf::Pref<units::degree_t> pivot;
  frc846::ntinf::Pref<units::degree_t> wrist;
  frc846::ntinf::Pref<units::inch_t> telescope;
};

struct SuperStructureReadings {};
struct SuperStructureTarget {};

class SuperStructureSubsystem
    : public frc846::robot::GenericSubsystem<SuperStructureReadings,
                                             SuperStructureTarget> {
 private:
  PTWSetpoint currentSetpoint;
  PTWSetpoint manualAdjustments;

  units::degree_t wrist_trim_amt = 0_deg;

 public:
  PivotSubsystem* pivot_;
  WristSubsystem* wrist_;
  TelescopeSubsystem* telescope_;

  SuperStructureSubsystem(PivotSubsystem* pivot, WristSubsystem* wrist,
                          TelescopeSubsystem* telescope)
      : frc846::robot::GenericSubsystem<SuperStructureReadings,
                                        SuperStructureTarget>{"SuperStructure",
                                                              true},
        pivot_{pivot},
        wrist_{wrist},
        telescope_{telescope} {
    currentSetpoint = {pivot_->pivot_home_offset_.value(), 0_in,
                       wrist_->wrist_home_offset_.value()};
  };

  void Setup() override;

  SuperStructureTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroSubsystem() {
    pivot_->ZeroSubsystem();
    telescope_->ZeroSubsystem();
    hasZeroed = true;
  }

  bool GetHasZeroed() { return hasZeroed; }

  void SetTargetSetpoint(PTWSetpoint newSetpoint) {
    if (currentSetpoint.pivot != newSetpoint.pivot ||
        currentSetpoint.wrist != newSetpoint.wrist ||
        currentSetpoint.telescope != newSetpoint.telescope) {
      currentSetpoint = newSetpoint;
      ClearAdjustments();
    }
  }

  bool CheckValidAdjustment(PTWSetpoint adjusted) {
    return true;
    return arm_kinematics_calculator.WithinBounds(
        arm_kinematics_calculator.calculate(
            {(currentSetpoint.pivot + adjusted.pivot),
             (currentSetpoint.telescope + adjusted.telescope),
             (currentSetpoint.wrist + adjusted.wrist)}));
  };

  void AdjustSetpoint(PTWSetpoint newAdj) {
    auto newAdjusted = newAdj + manualAdjustments;

    newAdjusted = {
        pivot_->CapWithinLimits(newAdjusted.pivot + currentSetpoint.pivot) -
            currentSetpoint.pivot,
        telescope_->CapWithinLimits(newAdjusted.telescope +
                                    currentSetpoint.telescope) -
            currentSetpoint.telescope,
        wrist_->CapWithinLimits(newAdjusted.wrist + currentSetpoint.wrist) -
            currentSetpoint.wrist};

    if (CheckValidAdjustment(newAdjusted)) {
      manualAdjustments = newAdjusted;
    }
  }

  void ClearAdjustments() {
    manualAdjustments = {0.0_deg, 0.0_in, wrist_trim_amt};
  }

  bool hasReachedSetpoint(PTWSetpoint setpoint) {
    return pivot_->WithinTolerance(setpoint.pivot) &&
           wrist_->WithinTolerance(setpoint.wrist) &&
           telescope_->WithinTolerance(setpoint.telescope);
  }

  /*
   */

  frc846::ntinf::Pref<units::degree_t> wrist_trim_inc{
      *this, "wrist_trim_increment", 0.5_deg};

  void TrimUp() {
    wrist_trim_amt += wrist_trim_inc.value();
    manualAdjustments.wrist += wrist_trim_inc.value();
  }

  void TrimDown() {
    wrist_trim_amt -= wrist_trim_inc.value();
    manualAdjustments.wrist -= wrist_trim_inc.value();
  }

  /*
   */

  frc846::ntinf::Pref<double> manual_control_deadband_{
      *this, "manual_control_deadband", 0.05};

  frc846::base::Loggable auto_named_{*this, "auto"};
  frc846::ntinf::Pref<units::second_t> post_shoot_wait_{auto_named_,
                                                        "post_shoot_wait", 1_s};
  frc846::ntinf::Pref<units::second_t> pre_shoot_wait_{auto_named_,
                                                       "pre_shoot_wait", 1_s};

  frc846::base::Loggable shooting_named_{*this, "shooting"};
  frc846::ntinf::Pref<units::inch_t> teleop_shooter_height_{
      shooting_named_, "teleop_shooter_height", 39_in};
  frc846::ntinf::Pref<units::inch_t> teleop_shooter_x_{
      shooting_named_, "teleop_shooter_x", 4_in};
  frc846::ntinf::Pref<units::inch_t> auto_shooter_height_{
      shooting_named_, "auto_shooter_height", 21_in};
  frc846::ntinf::Pref<units::inch_t> auto_shooter_x_{shooting_named_,
                                                     "auto_shooter_x", 13_in};

  // For shooter positions
  frc846::base::Loggable intake_point_arm_pose_{*this, "intake_point_arm_pose"};

  frc846::ntinf::Grapher<units::inch_t> intake_point_x_graph_{
      intake_point_arm_pose_, "x"};
  frc846::ntinf::Grapher<units::inch_t> intake_point_y_graph_{
      intake_point_arm_pose_, "y"};

  frc846::base::Loggable shooter_point_arm_pose_{*this,
                                                 "shooter_point_arm_pose"};
  frc846::ntinf::Grapher<units::inch_t> shooter_point_x_graph_{
      shooter_point_arm_pose_, "x"};
  frc846::ntinf::Grapher<units::inch_t> shooter_point_y_graph_{
      shooter_point_arm_pose_, "y"};

  /*
   */

  frc846::base::Loggable trap_named_{*this, "trap"};

  frc846::ntinf::Pref<units::degree_t> pivot_pull_target_{
      trap_named_, "pivot_pull_target", -10_deg};

  /*
   */

  // SETPOINTS
  PTWSetPref amp_position_{"amp_position", {0_deg, 0_in, 0_deg}};
  PTWSetpoint getAmpSetpoint() { return amp_position_.get(); }

  PTWSetPref intake_position_{"intake_position", {0_deg, 0_in, 0_deg}};
  PTWSetpoint getIntakeSetpoint() { return intake_position_.get(); }

  PTWSetPref source_position_{"source_position", {0_deg, 0_in, 0_deg}};
  PTWSetpoint getSourceSetpoint() { return source_position_.get(); }

  PTWSetPref stow_position_{"stow_position", {0_deg, 0_in, 0_deg}};
  PTWSetpoint getStowSetpoint() { return stow_position_.get(); }

  PTWSetPref shoot_position_{"shoot_position", {0_deg, 0_in, 0_deg}};
  PTWSetpoint getShootSetpoint() { return shoot_position_.get(); }

  PTWSetPref auto_shoot_position_{"auto_shoot_position", {0_deg, 0_in, 0_deg}};
  PTWSetpoint getAutoShootSetpoint() { return auto_shoot_position_.get(); }

  PTWSetPref preclimb_position_{"preclimb_position", {0_deg, 0_in, 0_deg}};
  PTWSetpoint getPreClimbSetpoint() { return preclimb_position_.get(); }

  PTWSetPref prescore_position_{"prescore_position", {0_deg, 0_in, 0_deg}};
  PTWSetpoint getPreScoreSetpoint() { return prescore_position_.get(); }

  PTWSetPref trapscore_position_{"trapscore_position", {0_deg, 0_in, 0_deg}};
  PTWSetpoint getTrapScoreSetpoint() { return trapscore_position_.get(); }

  PTWSetPref postscore_position_{"postscore_position", {0_deg, 0_in, 0_deg}};
  PTWSetpoint getPostScoreSetpoint() { return postscore_position_.get(); }

  PTWSetPref pass_position_{"pass_position", {40_deg, 0_in, 20_deg}};
  PTWSetpoint getPassSetpoint() { return pass_position_.get(); }

 private:
  bool hasZeroed = false;

  ArmKinematicsCalculator arm_kinematics_calculator;

  SuperStructureReadings ReadFromHardware() override;

  void WriteToHardware(SuperStructureTarget target) override;
};
