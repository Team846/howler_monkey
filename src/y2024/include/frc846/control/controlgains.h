#ifndef FRC846_CONTROL_GAINS_H_
#define FRC846_CONTROL_GAINS_H_

#include <rev/CANSparkMax.h>
#include <units/current.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "frc846/ctre_namespace.h"
#include "frc846/util/pref.h"
#include "units/time.h"

FRC846_CTRE_NAMESPACE()

namespace frc846::control {

struct ControlGains {
  double p;
  double i;
  double d;
  double f;
  double max_integral_accumulator;
};

class ControlGainsHelper : public Loggable {
 public:
  ControlGainsHelper(Loggable& parent, ControlGains gains,
                     units::ampere_t currentLimit = 40.0_A,
                     double peak_output_ = 1.0, double rampRate = 1.0);

  frc846::Pref<double> p_;
  frc846::Pref<double> i_;
  frc846::Pref<double> d_;
  frc846::Pref<double> f_;
  frc846::Pref<double> max_integral_accumulator_;
  frc846::Pref<double> current_limit_;
  frc846::Pref<double> peak_output_;
  frc846::Pref<double> reverse_peak_output_;

  frc846::Pref<double> ramp_rate_;

  void Write(ctre::phoenix6::configs::TalonFXConfigurator& configurator,
             ctre::phoenix6::configs::TalonFXConfiguration& configs,
             ControlGains& cache, bool ignore_cache = false);

  void Write(rev::SparkPIDController& pid_controller, ControlGains& cache,
             bool ignore_cache = false);

 private:
  static const int idx = 0;

  void UpdateCache(ControlGains& cache);
};

}  // namespace frc846::control

#endif  // FRC846_CONTROL_GAINS_H_