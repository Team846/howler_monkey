#pragma once

#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/voltage.h>

#include <algorithm>

#include "config.h"
#include "frc846/ntinf/pref.h"
#include "frc846/util/share_tables.h"

namespace frc846::motion {

class MotionTarget {
 public:
  static frc846::base::Loggable preferences_loggable;
};

struct CurrentControlSettings {
  units::ampere_t smart_current_limit;
  units::ampere_t motor_stall_current;

  double braking_constant;
};

class CurrentControl {
 public:
  CurrentControl(frc846::base::Loggable& parent,
                 CurrentControlSettings settings)
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

  frc846::ntinf::Pref<units::ampere_t> smart_current_limit_;
  frc846::ntinf::Pref<double> braking_constant_;
};

template <typename V>
class BrakingVelocityFPID {
 public:
  BrakingVelocityFPID(frc846::base::Loggable& parent,
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
  BrakingPositionDyFPID(frc846::base::Loggable& parent,
                        std::function<double(X)> prop_ff_function,
                        CurrentControlSettings current_control_settings,
                        frc846::control::HardLimitsConfigHelper<X>* hard_limits)
      : prop_ff_function_{prop_ff_function},
        current_control_{parent, current_control_settings},
        hard_limits_{hard_limits} {}

  double calculate(X target_pos, X current_pos,
                   double current_velocity_percentage,  // Only using dyF, P, D
                   frc846::control::Gains g) {
    auto hard_limits_vals = hard_limits_->get();

    double target_pos_capped =
        std::max(std::min(hard_limits_vals.forward.template to<double>(),
                          target_pos.template to<double>()),
                 hard_limits_vals.reverse.template to<double>());

    double error = target_pos_capped - current_pos.template to<double>();

    double target_output = (g.kF * prop_ff_function_(current_pos) +
                            g.kP * error + g.kD * current_velocity_percentage);

    return current_control_.calculate(current_velocity_percentage,
                                      target_output);
  }

 private:
  std::function<double(X)> prop_ff_function_;

  CurrentControl current_control_;

  frc846::control::HardLimitsConfigHelper<X>* hard_limits_;
};

struct GenericMotionWaypoint {
  double time;
  double position;
};

/*
 * TIME CUT trapezoidal motion profile; targets are positions. Does not use
 * WPILib units, allowing it to be used more generally.
 */
// class TrapezoidalMotionProfile {
//  public:
//   /*
//    * Standard motion profile. Default time_cut is 0.02 (seconds), loop time
//    for
//    * a 50_Hz loop.
//    */
//   TrapezoidalMotionProfile(double target, double current, double max_vel,
//                            double max_acc, double time_cut = 0.02)
//       : profile{} {
//     sign_ = (target >= current) ? 1 : -1;

//     double dist = std::abs(target - current);

//     std::vector<GenericMotionWaypoint> local_motion{};
//     local_motion.push_back({0.0, 0.0});

//     double half_dist = dist / 2.0;

//     double dist_traversed = 0.0;
//     double current_velocity = 0.0;

//     double time{time_cut};
//     for (;; time += time_cut) {
//       double next_velocity =
//           std::min(current_velocity + max_acc * time_cut, max_vel);
//       double avg_velocity = (current_velocity + next_velocity) / 2.0;

//       dist_traversed += avg_velocity * time_cut;
//       if (dist_traversed >= half_dist) break;

//       local_motion.push_back({time, dist_traversed});

//       current_velocity = next_velocity;
//     }

//     int halfNumPoints = local_motion.size();
//     for (int i = halfNumPoints - 1; i >= 0; i++) {
//       auto w = local_motion.at(i);

//       local_motion.push_back({time * 2 - w.time, dist - w.position});
//     }

//     for (auto w : local_motion) {
//       profile.push_back({w.time, w.position * sign_ + current});
//     }
//   }

//   /*
//    * Finds time associated with current position. Adds time_step.
//    Interpolates
//    * using closest two waypoints in profile to find next target position.
//    *
//    * Time step is in seconds.
//    */
//   double getNextTarget(double current, double time_step = 0.02) {
//     if (profile.size() <= 4) return current;

//     GenericMotionWaypoint closest_above{-1.0, 0.0};

//     int index_found_at = -1;
//     for (int i = 0; i < profile.size(); i++) {
//       auto w = profile.at(i);
//       if ((sign_ == 1 && w.position >= current) ||
//           (sign_ == -1 && w.position <= current)) {
//         closest_above = w;
//         index_found_at = i;
//       }
//     }

//     if (index_found_at != -1) return profile.at(profile.size() - 1).position;

//     double time_found_at = closest_above.time;

//     if (index_found_at != 0) {
//       auto closest_beneath = profile.at(index_found_at - 1);

//       time_found_at =
//           closest_beneath.time +
//           (closest_above.time - closest_beneath.time) *
//               (std::abs(current - closest_beneath.position)) /
//               (std::abs(closest_above.position - closest_beneath.position));
//     }

//     double next_target_time = time_found_at + time_step;

//     // TODO: reverse interpolation process (time -> dist)

//     return 0.0;
//   }

//  private:
//   std::vector<GenericMotionWaypoint> profile;
//   int sign_ = 1;
// };

}  // namespace frc846::motion