#ifndef frcLib846_MOTOR_GAINS_H_
#define frcLib846_MOTOR_GAINS_H_

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/CANSparkMax.h>

#include "frcLib846/ctre_namespace.h"
#include "frcLib846/pref.h"
#include "units/time.h"

frcLib846_CTRE_NAMESPACE()

namespace frcLib846::motor {

struct Gains {
  double p;
  double i;
  double d;
  double f;
  double max_integral_accumulator;
};

class GainsHelper : public Loggable {
 public:
  GainsHelper(Loggable parent, Gains gains);

  frcLib846::Pref<double> p_;
  frcLib846::Pref<double> i_;
  frcLib846::Pref<double> d_;
  frcLib846::Pref<double> f_;
  frcLib846::Pref<double> max_integral_accumulator_;

  void Write(ctre::BaseTalon& esc, Gains& cache,
             units::time::millisecond_t timeout, bool ignore_cache = false);

  void Write(rev::SparkPIDController& pid_controller, Gains& cache,
             bool ignore_cache = false);

 private:
  static const int idx = 0;

  void UpdateCache(Gains& cache);
};

}  // namespace frcLib846::motor

#endif  // frcLib846_MOTOR_GAINS_H_