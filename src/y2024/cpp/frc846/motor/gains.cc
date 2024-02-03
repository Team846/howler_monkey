#include "frc846/motor/gains.h"

#include "frc846/motor/helper.h"

namespace frc846::motor {

GainsHelper::GainsHelper(Loggable parent, Gains gains)
    : Loggable{parent, "gains"},
      p_{*this, "p", gains.p},
      i_{*this, "i", gains.i},
      d_{*this, "d", gains.d},
      f_{*this, "f", gains.f},
      max_integral_accumulator_{*this, "max_integral_accumulator",
                                gains.max_integral_accumulator} {}

void GainsHelper::Write(ctre::BaseTalon& esc, Gains& cache,
                        units::time::millisecond_t timeout, bool ignore_cache) {
  double timeout_ms = timeout.to<double>();

  if (ignore_cache || cache.p != p_.value()) {
    auto err = esc.Config_kP(idx, p_.value(), timeout_ms);
    CheckOk(*this, err, "p");
  }
  if (ignore_cache || cache.i != i_.value()) {
    auto err = esc.Config_kI(idx, i_.value(), timeout_ms);
    CheckOk(*this, err, "i");
  }
  if (ignore_cache || cache.d != d_.value()) {
    auto err = esc.Config_kD(idx, d_.value(), timeout_ms);
    CheckOk(*this, err, "d");
  }
  if (ignore_cache || cache.f != f_.value()) {
    auto err = esc.Config_kF(idx, f_.value(), timeout_ms);
    CheckOk(*this, err, "f");
  }
  if (ignore_cache ||
      cache.max_integral_accumulator != max_integral_accumulator_.value()) {
    auto err = esc.Config_IntegralZone(idx, max_integral_accumulator_.value(),
                                       timeout_ms);
    CheckOk(*this, err, "max integral accumulator");
  }

  UpdateCache(cache);
}

void GainsHelper::Write(rev::SparkPIDController& pid_controller,
                        Gains& cache, bool ignore_cache) {
  if (ignore_cache || cache.p != p_.value()) {
    auto err = pid_controller.SetP(p_.value());
    CheckOk(*this, err, "p");
  }
  if (ignore_cache || cache.i != i_.value()) {
    auto err = pid_controller.SetI(i_.value());
    CheckOk(*this, err, "i");
  }
  if (ignore_cache || cache.d != d_.value()) {
    auto err = pid_controller.SetD(d_.value());
    CheckOk(*this, err, "d");
  }
  if (ignore_cache || cache.f != f_.value()) {
    auto err = pid_controller.SetFF(f_.value());
    CheckOk(*this, err, "f");
  }
  if (ignore_cache ||
      cache.max_integral_accumulator != max_integral_accumulator_.value()) {
    auto err = pid_controller.SetIMaxAccum(max_integral_accumulator_.value());
    CheckOk(*this, err, "max integral accumulator");
  }

  UpdateCache(cache);
}

void GainsHelper::UpdateCache(Gains& cache) {
  cache.p = p_.value();
  cache.i = i_.value();
  cache.d = d_.value();
  cache.f = f_.value();
  cache.max_integral_accumulator = max_integral_accumulator_.value();
}

}  // namespace frc846::motor