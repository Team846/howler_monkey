#ifndef CONTROL_CONFIG_H_
#define CONTROL_CONFIG_H_

#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "frc846/subsystem.h"
#include "frc846/util/pref.h"

namespace frc846::control {

struct DefaultSpecifications {
  static constexpr units::ampere_t stall_current_kraken = 366_A;
  static constexpr units::ampere_t stall_current_vortex = 211_A;
  static constexpr units::ampere_t stall_current_neo = 105_A;
  static constexpr units::ampere_t stall_current_550 = 100_A;

  static constexpr units::revolutions_per_minute_t free_speed_kraken = 6000_rpm;
  static constexpr units::revolutions_per_minute_t free_speed_vortex = 6784_rpm;
  static constexpr units::revolutions_per_minute_t free_speed_neo = 5676_rpm;
  static constexpr units::revolutions_per_minute_t free_speed_550 = 11000_rpm;
};

struct CurrentLimiting {
  units::ampere_t target_threshold;
  units::millisecond_t peak_time_threshold = 200_ms;
};

class CurrentLimitingPrefs {
 public:
  CurrentLimitingPrefs(Loggable& parent, CurrentLimiting default_limiting)
      : rep{parent, "CurrentLimiting"},
        target_threshold{rep, "target_threshold",
                         default_limiting.target_threshold},
        peak_time_theshold{rep, "peak_time_threshold",
                           default_limiting.peak_time_threshold} {}

  CurrentLimiting get() {
    return {target_threshold.value(), peak_time_theshold.value()};
  }

 private:
  Loggable rep;
  frc846::Pref<units::ampere_t> target_threshold;
  frc846::Pref<units::millisecond_t> peak_time_theshold;
};

struct Gains {
  double kP;
  double kD;

  double kF = 0.0;
};

class GainsPrefs {
 public:
  GainsPrefs(Loggable& parent, Gains default_gains)
      : rep{parent, "Gains"},
        kP{rep, "kP", default_gains.kP},
        kD{rep, "kD", default_gains.kD},
        kF{rep, "kF", default_gains.kF} {}

  Gains get() { return {kP.value(), kD.value(), kF.value()}; }

 private:
  Loggable rep;
  frc846::Pref<double> kP;
  frc846::Pref<double> kD;
  frc846::Pref<double> kF;
};

enum MotorIdleMode { kDefaultCoast, kDefaultBrake };

// struct FFCalibrationPoint {
//   double position;
//   double duty_cycle;
// };

// struct FFTable {
//   std::vector<FFCalibrationPoint> ff_calibration_points;
// };

template <class X>
struct HardLimits {
  X forward;
  X reverse;
  bool usingPositionLimits;

  double peak_output_forward = 1.0;
  double peak_output_reverse = -1.0;
};

template <class X>
class HardLimitsConfigHelper {
 public:
  HardLimitsConfigHelper(Loggable& parent, HardLimits<X> default_limits)
      : rep{parent, Loggable::Join("Configs", "HardLimits")},
        forward_{rep, "forward", default_limits.forward},
        reverse_{rep, "reverse", default_limits.reverse},
        using_position_limits_{rep, "use_position_limits",
                               default_limits.usingPositionLimits},
        peak_output_forward_{rep, "peak_output_forward",
                             default_limits.peak_output_forward},
        peak_output_reverse_{rep, "peak_output_reverse",
                             default_limits.peak_output_reverse} {}

  HardLimits<X> get() {
    return {forward_.value(), reverse_.value(), using_position_limits_.value(),
            peak_output_forward_.value(), peak_output_reverse_.value()};
  }

 private:
  Loggable rep;
  frc846::Pref<X> forward_;
  frc846::Pref<X> reverse_;
  frc846::Pref<bool> using_position_limits_;

  frc846::Pref<double> peak_output_forward_;
  frc846::Pref<double> peak_output_reverse_;
};

enum DataTag {
  kPositionData,
  kVelocityData,
  kCurrentData,
  kFaultData,
  kSensorData,
  kLeader
};

struct MotorConfig {
  bool invert = false;

  double gear_ratio = 1.0;

  MotorIdleMode idle_mode = kDefaultCoast;

  CurrentLimiting current_limiting = {40_A};

  units::second_t rampTime = 0.0_s;
  bool withRampRate = false;

  units::volt_t voltage_compensation = 16.0_V;
  units::volt_t auton_voltage_compensation = 12.0_V;
};

class MotorConfigPrefs {
 public:
  MotorConfigPrefs(Loggable& parent, MotorConfig default_config)
      : rep{parent, "Configs"},
        invert_{rep, "invert", default_config.invert},
        gear_ratio_{rep, "gear_ratio", default_config.gear_ratio},
        default_brake_{rep, "default_brake_mode",
                       default_config.idle_mode == kDefaultBrake},
        current_limiting_{rep, default_config.current_limiting},
        ramp_time_{rep, "ramp_time", default_config.rampTime},
        using_ramp_rate_{rep, "use_ramp_rate", default_config.withRampRate},
        voltage_compensation_{rep, "voltage_compensation",
                              default_config.voltage_compensation},
        auton_voltage_compensation_{rep, "auton_voltage_compensation",
                                    default_config.auton_voltage_compensation} {
  }

  MotorConfig get() {
    return MotorConfig{invert_.value(),
                       gear_ratio_.value(),
                       default_brake_.value() ? kDefaultBrake : kDefaultCoast,
                       current_limiting_.get(),
                       ramp_time_.value(),
                       using_ramp_rate_.value(),
                       voltage_compensation_.value(),
                       auton_voltage_compensation_.value()};
  }

 private:
  Loggable rep;
  frc846::Pref<bool> invert_;
  frc846::Pref<double> gear_ratio_;
  frc846::Pref<bool> default_brake_;
  CurrentLimitingPrefs current_limiting_;
  frc846::Pref<units::second_t> ramp_time_;
  frc846::Pref<bool> using_ramp_rate_;
  frc846::Pref<units::volt_t> voltage_compensation_;
  frc846::Pref<units::volt_t> auton_voltage_compensation_;
};

class ConfigHelper {
 public:
  ConfigHelper(Loggable& parent, MotorConfig config, Gains default_gains)
      : parent_{parent},
        motor_config_{parent, config},
        gains_{parent, default_gains} {
    cachedGains = gains_.get();
  }

  Gains getGains() { return cachedGains; };

  bool hasGainsChanged() {
    auto latestGains = gains_.get();
    if (!deq(latestGains.kP, cachedGains.kP) ||
        !deq(latestGains.kD, cachedGains.kD) ||
        !deq(latestGains.kF, cachedGains.kF)) {
      cachedGains = latestGains;
      return true;
    }
    return false;
  }

  Gains updateAndGetGains() {
    hasGainsChanged();
    return getGains();
  }

  MotorConfig getMotorConfig() { return motor_config_.get(); };

 private:
  bool deq(double a, double b) { return std::abs(a - b) <= 0.01; }

  Loggable& parent_;
  MotorConfigPrefs motor_config_;
  GainsPrefs gains_;

  Gains cachedGains;
};

}  // namespace frc846::control

#endif