#ifndef FRC846_MOTOR_HELPER_H_
#define FRC846_MOTOR_HELPER_H_

#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/CANSparkMax.h>

#include <initializer_list>

#include "frc846/ctre_namespace.h"
#include "frc846/motor/config.h"
#include "frc846/motor/gains.h"

FRC846_CTRE_NAMESPACE()

namespace frc846::motor {

// Default CAN timeout for speed controllers.
static constexpr units::millisecond_t kCANTimeout = 50_ms;

// Motor output control mode.
enum ControlMode { Percent, Velocity, Position, Current };

// Convert control mode to ctre control mode.
// constexpr ctre::ControlMode CTREControlMode(ControlMode mode);

// Convert control mode to rev control type.
constexpr rev::CANSparkMax::ControlType RevControlMode(ControlMode mode);

// Check ctre status code ok.
// void CheckOk(const Loggable loggable, ctre::StatusCode err, std::string_view field);

// Check rev status code ok.
void CheckOk(const Loggable loggable, rev::REVLibError err, std::string_view field);

// Motor output.
struct Output {
  ControlMode mode;
  double value;

  bool operator==(const Output& other) {
    return mode == other.mode && value == other.value;
  }

  bool operator!=(const Output& other) { return !(*this == other); }
};


class TalonFXHelper {
 public:
  TalonFXHelper(Loggable parent, ctre::phoenix6::hardware::TalonFX& esc,
                TalonFXConfigHelper* config, GainsHelper* gains);

  ~TalonFXHelper();

  void Setup(units::millisecond_t timeout = kCANTimeout);

  bool VerifyConnected();

  void OnInit(std::function<void()> callback);

  void Write(Output output, units::millisecond_t timeout = kCANTimeout);

 private:
  Loggable parent_;

  ctre::phoenix6::hardware::TalonFX& esc_;

  std::vector<std::function<void()>> on_inits_;

  TalonFXConfigHelper* config_;
  GainsHelper* gains_;

  TalonFXConfig config_cache_;
  Gains gains_cache_;
};

class SparkMAXHelper {
 public:
  SparkMAXHelper(Loggable parent, rev::CANSparkMax& esc,
                 SparkMAXConfigHelper* config, GainsHelper* gains);

  ~SparkMAXHelper();

  rev::SparkPIDController& pid_controller() { return pid_controller_; }

  rev::SparkRelativeEncoder& encoder() { return encoder_; }

  void Setup(units::millisecond_t timeout = kCANTimeout);
  void Setup(bool brake, units::millisecond_t timeout = kCANTimeout);

  void DisableStatusFrames(
      std::initializer_list<rev::CANSparkLowLevel::PeriodicFrame> frames);

  bool VerifyConnected();

  void OnInit(std::function<void()> callback);

  void Write(Output output);

 private:
  Loggable parent_;

  rev::CANSparkMax& esc_;
  rev::SparkPIDController pid_controller_;
  rev::SparkRelativeEncoder encoder_;

  std::vector<std::function<void()>> on_inits_;

  SparkMAXConfigHelper* config_;
  GainsHelper* gains_;

  SparkMAXConfig config_cache_;
  Gains gains_cache_;
};

}  // namespace frc846::motor

#endif  // FRC846_MOTOR_HELPER_H_