#ifndef frcLib846_MOTOR_CONFIG_H_
#define frcLib846_MOTOR_CONFIG_H_

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/CANSparkMax.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "frcLib846/ctre_namespace.h"
#include "frcLib846/pref.h"

frcLib846_CTRE_NAMESPACE()

namespace frcLib846::motor {

// Base configurations for VictorSPX speed controllers.
struct VictorSPXConfig {
  double peak_output;  // [0, 1]
  units::volt_t voltage_comp_saturation;
};

// Base configurations for TalonSRX speed controllers.
struct TalonSRXConfig {
  double peak_output;  // [0, 1]
  units::volt_t voltage_comp_saturation;

  units::ampere_t peak_current_limit;
  units::millisecond_t peak_current_duration;
  units::ampere_t continuous_current_limit;
};

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

// Configuration helper for VictorSPX speed controllers.
//
// Creates preferences and write to speed controller.
class VictorSPXConfigHelper : public Loggable {
 public:
  VictorSPXConfigHelper(Loggable parent, VictorSPXConfig config);

  frcLib846::Pref<double> peak_output_;
  frcLib846::Pref<units::volt_t> voltage_comp_saturation_;

  // Write configuration to speed controller.
  void Write(ctre::VictorSPX& esc, VictorSPXConfig& cache,
             units::millisecond_t timeout, bool ignore_cache = false);
};

// Configuration helper for TalonSRX speed controllers.
//
// Creates preferences and write to speed controller.
class TalonSRXConfigHelper : public Loggable {
 public:
  TalonSRXConfigHelper(Loggable parent, TalonSRXConfig config);

  frcLib846::Pref<double> peak_output_;
  frcLib846::Pref<units::volt_t> voltage_comp_saturation_;

  frcLib846::Pref<units::ampere_t> peak_current_limit_;
  frcLib846::Pref<units::millisecond_t> peak_current_duration_;
  frcLib846::Pref<units::ampere_t> continuous_current_limit_;

  // Write configuration to speed controller.
  void Write(ctre::TalonSRX& esc, TalonSRXConfig& cache,
             units::millisecond_t timeout, bool ignore_cache = false);
};

// Configuration helper for TalonFX speed controllers.
//
// Creates preferences and write to speed controller.
class TalonFXConfigHelper : public Loggable {
 public:
  TalonFXConfigHelper(Loggable parent, TalonFXConfig config);

  frcLib846::Pref<double> peak_output_;
  frcLib846::Pref<units::volt_t> voltage_comp_saturation_;

  frcLib846::Pref<units::ampere_t> supply_peak_current_limit_;
  frcLib846::Pref<units::millisecond_t> supply_peak_current_duration_;
  frcLib846::Pref<units::ampere_t> supply_continuous_current_limit_;

  frcLib846::Pref<units::ampere_t> stator_peak_current_limit_;
  frcLib846::Pref<units::millisecond_t> stator_peak_current_duration_;
  frcLib846::Pref<units::ampere_t> stator_continuous_current_limit_;

  // Write configuration to speed controller.
  void Write(ctre::TalonFX& esc, TalonFXConfig& cache,
             units::millisecond_t timeout, bool ignore_cache = false);
};

// Configuration helper for SparkMAX speed controllers.
//
// Creates preferences and write to speed controller.
class SparkMAXConfigHelper : public Loggable {
 public:
  SparkMAXConfigHelper(Loggable parent, SparkMAXConfig config);

  frcLib846::Pref<double> peak_output_;
  frcLib846::Pref<units::volt_t> voltage_comp_saturation_;

  frcLib846::Pref<units::ampere_t> current_limit_;

  void Write(rev::CANSparkMax& esc, rev::SparkPIDController& pid_controller,
             SparkMAXConfig& cache, bool ignore_cache = false);
};

}  // namespace frcLib846::motor

#endif  // frcLib846_MOTOR_CONFIG_H_