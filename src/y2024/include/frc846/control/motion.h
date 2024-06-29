#ifndef FRC846_MOTION_H_
#define FRC846_MOTION_H_

#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/voltage.h>

#include "config.h"
#include "frc846/util/pref.h"
#include "frc846/util/share_tables.h"

namespace frc846::motion {

class MotionTarget {
 public:
  static frc846::Loggable preferences_loggable;
};

struct CurrentControlSettings {
  units::ampere_t smart_current_limit;
  units::ampere_t motor_stall_current;

  double braking_constant;
};

class CurrentControl {
 public:
  CurrentControl(frc846::Loggable& parent, CurrentControlSettings settings)
      : settings_{settings},
        smart_current_limit_{parent, "smart_current_limit",
                             settings.smart_current_limit},
        braking_constant_{parent, "braking_p", settings.braking_constant} {}

  double calculate(double current_velocity_percentage,
                   double target_velocity_percentage) {
    double output = current_velocity_percentage;

    double braking_force =
        (target_velocity_percentage - current_velocity_percentage) *
        braking_constant_.value();

    output += braking_force;

    units::volt_t generatedEMF = 12.0_V * current_velocity_percentage;

    units::volt_t upperTargetEMF = (12.0_V * smart_current_limit_.value() /
                                    settings_.motor_stall_current) +
                                   generatedEMF;
    units::volt_t lowerTargetEMF = -(12.0_V * smart_current_limit_.value() /
                                     settings_.motor_stall_current) +
                                   generatedEMF;

    double upperDCBound = upperTargetEMF / 12_V;
    double lowerDCBound = lowerTargetEMF / 12_V;

    output = std::min(upperDCBound, std::max(lowerDCBound, output));

    return output;
  }

 private:
  CurrentControlSettings settings_;

  frc846::Pref<units::ampere_t> smart_current_limit_;
  frc846::Pref<double> braking_constant_;
};

template <typename V>
class BrakingVelocityFPID {
 public:
  BrakingVelocityFPID(frc846::Loggable& parent,
                      CurrentControlSettings current_control_settings)
      : current_control_{parent, current_control_settings} {}

  double calculate(V target_velocity, V current_velocity,
                   double current_velocity_percentage,  // Only using F and P
                   frc846::control::Gains g) {
    double error = target_velocity.template to<double>() -
                   current_velocity.template to<double>();

    double target_output =
        g.kF * target_velocity.template to<double>() + g.kP * error;

    return current_control_.calculate(current_velocity_percentage,
                                      target_output);
  }

 private:
  CurrentControl current_control_;
};

template <typename X>
class BrakingPositionDyFPID {
  using V = units::unit_t<
      units::compound_unit<typename X::unit_type,  // the velocity unit is equal
                                                   // to the position unit / 1_s
                           units::inverse<units::second>>>;

 public:
  BrakingPositionDyFPID(Loggable& parent,
                        std::function<double(X)> prop_ff_function,
                        CurrentControlSettings current_control_settings)
      : prop_ff_function_{prop_ff_function},
        current_control_{parent, current_control_settings} {}

  double calculate(X target_pos, X current_pos,
                   double current_velocity_percentage,  // Only using dyF, P, D
                   frc846::control::Gains g) {
    double error =
        target_pos.template to<double>() - current_pos.template to<double>();

    double target_output = g.kF * prop_ff_function_(current_pos) +
                           g.kP * error + g.kD * current_velocity_percentage;

    return current_control_.calculate(current_velocity_percentage,
                                      target_output);
  }

 private:
  std::function<double(X)> prop_ff_function_;

  CurrentControl current_control_;
};

}  // namespace frc846::motion

#endif  // FRC846_MOTION_H_