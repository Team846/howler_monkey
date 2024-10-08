#pragma once

#include "frc846/base/loggable.h"
#include "frc846/control/control.h"
#include "frc846/control/motion.h"
#include "frc846/ntinf/grapher.h"
#include "frc846/ntinf/pref.h"
#include "frc846/robot/GenericSubsystem.h"
#include "ports.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/math.h"

struct WristReadings {
  units::degree_t wrist_position;
  units::degrees_per_second_t wrist_velocity;
};

struct WristTarget {
  std::variant<units::degree_t, double> wrist_output;
  bool override_limits = false;
};

class WristSubsystem
    : public frc846::robot::GenericSubsystem<WristReadings, WristTarget> {
 public:
  bool hasZeroed = false;

  WristSubsystem(bool init);

  void Setup() override;

  WristTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool GetHasZeroed() { return hasZeroed; }

  bool WithinTolerance(units::degree_t pos) {
    return (units::math::abs(pos - GetReadings().wrist_position) <
            wrist_tolerance_.value());
  }

  void Coast() {
    if (auto esc = wrist_esc_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  }

  void Brake() {
    if (auto esc = wrist_esc_.getESC())
      esc->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

  void DeZero() { hasZeroed = false; }

  void ZeroSubsystem() {
    hasZeroed = true;
    wrist_esc_.ZeroEncoder(wrist_home_offset_.value());
    SetTarget(ZeroTarget());
  }

  bool WithinLimits(units::degree_t pos) {
    return hard_limits_.WithinLimits(pos);
  }

  units::degree_t CapWithinLimits(units::degree_t pos) {
    return hard_limits_.CapWithinLimits(pos);
  }

  frc846::ntinf::Pref<units::degree_t> wrist_tolerance_{
      *this, "wrist_tolerance", 1.2_deg};

  frc846::ntinf::Pref<units::degree_t> wrist_home_offset_{
      *this, "home_offset_wrist", -49_deg};
  frc846::ntinf::Pref<units::degree_t> wrist_cg_offset_{
      *this, "cg_offset_wrist", 60_deg};

  frc846::ntinf::Pref<units::degrees_per_second_t> max_adjustment_rate_{
      *this, "max_adjustment_rate", units::degrees_per_second_t(80.0)};

  frc846::ntinf::Pref<units::degrees_per_second_t> homing_velocity_tolerance_{
      *this, "homing_velocity_tolerance", 1.0_deg_per_s};
  frc846::ntinf::Pref<double> num_loops_homed_{*this, "num_loops_homed", 20};
  frc846::ntinf::Pref<double> homing_speed_{*this, "homing_speed", -0.2};
  frc846::ntinf::Pref<units::degrees_per_second_t> homing_max_speed_{
      *this, "homing_max_speed", 30.0_deg_per_s};
  frc846::ntinf::Pref<double> homing_dc_cut_{*this, "homing_dc_cut", 1.5};

 private:
  frc846::base::Loggable gains_{*this, "gains"};
  frc846::ntinf::Pref<double> k_{gains_, "k", 0.0};
  frc846::ntinf::Pref<double> p_{gains_, "p", 0.0};
  frc846::ntinf::Pref<double> d_{gains_, "d", 0.0};

  frc846::base::Loggable readings_named_{*this, "readings"};

  frc846::ntinf::Grapher<units::degree_t> wrist_pos_graph{target_named_,
                                                          "wrist_pos"};
  frc846::ntinf::Grapher<units::degree_t> wrist_error_graph{target_named_,
                                                            "wrist_error"};

  frc846::ntinf::Grapher<units::degree_t> wrist_weight_pos_graph{
      target_named_, "wrist_weight_position"};

  frc846::base::Loggable target_named_{*this, "target"};

  frc846::ntinf::Grapher<double> target_wrist_duty_cycle_graph{
      target_named_, "wrist_duty_cycle"};

  frc846::ntinf::Grapher<units::degree_t> target_wrist_pos_graph{target_named_,
                                                                 "wrist_pos"};

  frc846::control::ConfigHelper config_helper_{
      *this,
      {false,
       3.0 / 250.0 * (360.0),
       frc846::control::MotorIdleMode::kDefaultBrake,
       {40_A}},
      {0.3, 0.00, 0.00}};

  frc846::control::HardLimitsConfigHelper<units::degree_t> hard_limits_{
      *this, {150_deg, 0_deg, true, 0.7, -0.5}};

  frc846::control::REVSparkController<units::degree_t> wrist_esc_{
      *this, ports::positioning_::kWrist_CANID, config_helper_, hard_limits_};

  WristReadings ReadFromHardware() override;

  void WriteToHardware(WristTarget target) override;

  frc846::base::Loggable dyFPID_loggable{*this, "DynamicFPID"};

  frc846::control::GainsPrefs gains_prefs_dyFPID{dyFPID_loggable,
                                                 {0.0, 0.0, 0.0}};

  frc846::motion::BrakingPositionDyFPID<units::degree_t> dyFPID{
      dyFPID_loggable,
      [&](units::degree_t pos) -> double {
        return units::math::sin(
                   1_deg *
                       frc846::util::ShareTables::GetDouble("pivot_position") -
                   GetReadings().wrist_position + wrist_cg_offset_.value())
            .to<double>();
      },
      {30_A, frc846::control::DefaultSpecifications::stall_current_neo, 0.3},
      &hard_limits_};

  // frc846::base::Loggable close_dyFPID_loggable{*this, "CloseDynamicFPID"};

  // frc846::motion::BrakingPositionDyFPID<units::degree_t> dyFPIDClose{
  //     close_dyFPID_loggable,
  //     [this](units::degree_t pos) -> double {
  //       return std::abs(
  //           units::math::cos(
  //               1_deg *
  //               frc846::util::ShareTables::GetDouble("pivot_position") -
  //               GetReadings().wrist_position + wrist_cg_offset_.value())
  //               .to<double>());
  //     },
  //     {15_A, frc846::control::DefaultSpecifications::stall_current_neo,
  //     0.3}};
};
