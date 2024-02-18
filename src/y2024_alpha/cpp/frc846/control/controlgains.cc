#include "frc846/control/controlgains.h"

namespace frc846::control {

void GCheckOk(Loggable& loggable, rev::REVLibError err, std::string field = "?") {
  if (err != rev::REVLibError::kOk) {
    loggable.Warn("Unable to update {}", field);
  }
};

void GCheckOk(Loggable& loggable, ctre::phoenix::StatusCode err, std::string field = "TalonFX Config") {
  if (!(err.IsOK() || err.IsWarning())) {
    loggable.Warn("Unable to update {}", field);
  }
};

ControlGainsHelper::ControlGainsHelper(Loggable& parent, ControlGains gains, units::ampere_t currentLimit, double peakOutput)
    : Loggable{parent, "gains"},
      p_{*this, "p", gains.p},
      i_{*this, "i", gains.i},
      d_{*this, "d", gains.d},
      f_{*this, "f", gains.f},
      max_integral_accumulator_{*this, "max_integral_accumulator",
                                gains.max_integral_accumulator},
      current_limit_{*this, "smart_current_limit", currentLimit.to<double>()},
      peak_output_{*this, "peak_output", peakOutput} {}

void ControlGainsHelper::Write(rev::SparkPIDController& pid_controller,
                        ControlGains& cache, bool ignore_cache) {
  if (ignore_cache || cache.p != p_.value()) {
    auto err = pid_controller.SetP(p_.value());
    GCheckOk(*this, err, "p");
  }
  if (ignore_cache || cache.i != i_.value()) {
    auto err = pid_controller.SetI(i_.value());
    GCheckOk(*this, err, "i");
  }
  if (ignore_cache || cache.d != d_.value()) {
    auto err = pid_controller.SetD(d_.value());
    GCheckOk(*this, err, "d");
  }
  if (ignore_cache || cache.f != f_.value()) {
    auto err = pid_controller.SetFF(f_.value());
    GCheckOk(*this, err, "f");
  }
  if (ignore_cache ||
      cache.max_integral_accumulator != max_integral_accumulator_.value()) {
    auto err = pid_controller.SetIMaxAccum(max_integral_accumulator_.value());
    GCheckOk(*this, err, "max integral accumulator");
  }

  UpdateCache(cache);
}

void ControlGainsHelper::UpdateCache(ControlGains& cache) {
  cache.p = p_.value();
  cache.i = i_.value();
  cache.d = d_.value();
  cache.f = f_.value();
  cache.max_integral_accumulator = max_integral_accumulator_.value();
}

}  // namespace frc846::control