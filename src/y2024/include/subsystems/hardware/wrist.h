#pragma once

#include "frc846/control/control.h"
#include "frc846/control/motion.h"
#include "frc846/loggable.h"
#include "frc846/subsystem.h"
#include "frc846/util/grapher.h"
#include "frc846/util/pref.h"
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

class WristSubsystem : public frc846::Subsystem<WristReadings, WristTarget> {
 public:
  WristSubsystem(bool init);

  void Setup() override;

  WristTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool GetHasZeroed() { return hasZeroed; }

  bool WithinTolerance(units::degree_t pos) {
    return (units::math::abs(pos - readings().wrist_position) <
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

  frc846::Pref<units::degree_t> wrist_tolerance_{*this, "wrist_tolerance",
                                                 1.2_deg};

  frc846::Pref<units::degree_t> wrist_home_offset_{*this, "home_offset_wrist",
                                                   -49_deg};
  frc846::Pref<units::degree_t> wrist_cg_offset_{*this, "cg_offset_wrist",
                                                 60_deg};

  frc846::Pref<units::degrees_per_second_t> max_adjustment_rate_{
      *this, "max_adjustment_rate", units::degrees_per_second_t(80.0)};

  frc846::Pref<units::degrees_per_second_t> homing_velocity_tolerance_{
      *this, "homing_velocity_tolerance", 1.0_deg_per_s};
  frc846::Pref<int> num_loops_homed_{*this, "num_loops_homed", 7};
  frc846::Pref<double> homing_speed_{*this, "homing_speed", -0.2};

 private:
  bool hasZeroed = false;

  frc846::Loggable gains_{*this, "gains"};
  frc846::Pref<double> k_{gains_, "k", 0.0};
  frc846::Pref<double> p_{gains_, "p", 0.0};
  frc846::Pref<double> d_{gains_, "d", 0.0};

  frc846::Loggable readings_named_{*this, "readings"};

  frc846::Grapher<units::degree_t> wrist_pos_graph{target_named_, "wrist_pos"};
  frc846::Grapher<units::degree_t> wrist_error_graph{target_named_,
                                                     "wrist_error"};

  frc846::Loggable target_named_{*this, "target"};

  frc846::Grapher<double> target_wrist_duty_cycle_graph{target_named_,
                                                        "wrist_duty_cycle"};

  frc846::Grapher<units::degree_t> target_wrist_pos_graph{target_named_,
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

  WristReadings GetNewReadings() override;

  void DirectWrite(WristTarget target) override;

  frc846::Loggable dyFPID_loggable{*this, "DynamicFPID"};

  frc846::motion::BrakingPositionDyFPID<units::degree_t> dyFPID{
      dyFPID_loggable,
      [this](units::degree_t pos) -> double {
        return std::abs(
            units::math::cos(
                1_deg * frc846::util::ShareTables::GetDouble("pivot_position") +
                readings().wrist_position - wrist_cg_offset_.value())
                .to<double>());
      },
      {30_A, frc846::control::DefaultSpecifications::stall_current_neo, 0.3}};
};
