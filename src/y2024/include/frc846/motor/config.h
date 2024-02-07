#ifndef FRC846_MOTOR_CONFIG_H_
#define FRC846_MOTOR_CONFIG_H_

#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/CANSparkMax.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "frc846/ctre_namespace.h"
#include "frc846/pref.h"

FRC846_CTRE_NAMESPACE()

namespace frc846::motor {

// Base configurations for TalonFX speed controllers.
struct TalonFXConfig {
  double peak_output;  // [0, 1]
  units::volt_t voltage_comp_saturation;

  units::ampere_t supply_peak_current_limit;
  units::millisecond_t supply_peak_current_duration;
  units::ampere_t supply_continuous_current_limit;

  units::ampere_t stator_peak_current_limit;
  units::millisecond_t stator_peak_current_duration;
  units::ampere_t stator_continuous_current_limit;
};

// Base configurations for Spark MAX speed controllers.
struct SparkMAXConfig {
  double peak_output;  // [0, 1]
  units::volt_t voltage_comp_saturation;

  units::ampere_t current_limit;
};

// Configuration helper for TalonFX speed controllers.
//
// Creates preferences and write to speed controller.
class TalonFXConfigHelper : public Loggable {
 public:
  TalonFXConfigHelper(Loggable parent, TalonFXConfig config);

  frc846::Pref<double> peak_output_;
  frc846::Pref<units::volt_t> voltage_comp_saturation_;

  frc846::Pref<units::ampere_t> supply_peak_current_limit_;
  frc846::Pref<units::millisecond_t> supply_peak_current_duration_;
  frc846::Pref<units::ampere_t> supply_continuous_current_limit_;

  frc846::Pref<units::ampere_t> stator_peak_current_limit_;
  frc846::Pref<units::millisecond_t> stator_peak_current_duration_;
  frc846::Pref<units::ampere_t> stator_continuous_current_limit_;

  // Write configuration to speed controller.
  void Write(ctre::phoenix6::hardware::TalonFX& esc, TalonFXConfig& cache,
             units::millisecond_t timeout, bool ignore_cache = false);
};

// Configuration helper for SparkMAX speed controllers.
//
// Creates preferences and write to speed controller.
class SparkMAXConfigHelper : public Loggable {
 public:
  SparkMAXConfigHelper(Loggable parent, SparkMAXConfig config);

  frc846::Pref<double> peak_output_;
  frc846::Pref<units::volt_t> voltage_comp_saturation_;

  frc846::Pref<units::ampere_t> current_limit_;

  void Write(rev::CANSparkMax& esc, rev::SparkPIDController& pid_controller,
             SparkMAXConfig& cache, bool ignore_cache = false);
};

}  // namespace frc846::motor

#endif  // FRC846_MOTOR_CONFIG_H_