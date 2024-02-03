#ifndef frcLib846_MOTOR_HELPER_H_
#define frcLib846_MOTOR_HELPER_H_

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/CANSparkMax.h>

#include <initializer_list>

#include "frcLib846/ctre_namespace.h"
#include "frcLib846/motor/config.h"
#include "frcLib846/motor/gains.h"

frcLib846_CTRE_NAMESPACE()

namespace frcLib846::motor {

// Default CAN timeout for speed controllers.
static constexpr units::millisecond_t kCANTimeout = 50_ms;

// Motor output control mode.
enum ControlMode { Percent, Velocity, Position, Current };

// Convert control mode to ctre control mode.
constexpr ctre::ControlMode CTREControlMode(ControlMode mode);

// Convert control mode to rev control type.
constexpr rev::CANSparkMax::ControlType RevControlMode(ControlMode mode);

// Check ctre status code ok.
void CheckOk(const Loggable loggable, ctre::ErrorCode err, std::string_view field);

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

class VictorSPXHelper {
 public:
  VictorSPXHelper(Loggable parent, ctre::VictorSPX& esc,
                  VictorSPXConfigHelper* config);

  ~VictorSPXHelper();

  void Setup(units::millisecond_t timeout = kCANTimeout);

  void DisableStatusFrames(
      std::initializer_list<ctre::StatusFrameEnhanced> frames,
      units::millisecond_t timeout = kCANTimeout);

  bool VerifyConnected();

  void OnInit(std::function<void()> callback);

  void Write(Output output, units::millisecond_t timeout = kCANTimeout);

 private:
  Loggable parent_;

  ctre::VictorSPX& esc_;

  std::vector<std::function<void()>> on_inits_;

  VictorSPXConfigHelper* config_;

  VictorSPXConfig config_cache_;
};

class TalonSRXHelper {
 public:
  TalonSRXHelper(Loggable parent, ctre::TalonSRX& esc,
                 TalonSRXConfigHelper* config, GainsHelper* gains);

  ~TalonSRXHelper();

  void Setup(units::millisecond_t timeout = kCANTimeout);

  void DisableStatusFrames(
      std::initializer_list<ctre::StatusFrameEnhanced> frames,
      units::millisecond_t timeout = kCANTimeout);

  bool VerifyConnected();

  void OnInit(std::function<void()> callback);

  void Write(Output output, units::millisecond_t timeout = kCANTimeout);

 private:
  Loggable parent_;

  ctre::TalonSRX& esc_;

  std::vector<std::function<void()>> on_inits_;

  TalonSRXConfigHelper* config_;
  GainsHelper* gains_;

  TalonSRXConfig config_cache_;
  Gains gains_cache_;
};

class TalonFXHelper {
 public:
  TalonFXHelper(Loggable parent, ctre::TalonFX& esc,
                TalonFXConfigHelper* config, GainsHelper* gains);

  ~TalonFXHelper();

  void Setup(units::millisecond_t timeout = kCANTimeout);

  void DisableStatusFrames(
      std::initializer_list<ctre::StatusFrameEnhanced> frames,
      units::millisecond_t timeout = kCANTimeout);

  bool VerifyConnected();

  void OnInit(std::function<void()> callback);

  void Write(Output output, units::millisecond_t timeout = kCANTimeout);

 private:
  Loggable parent_;

  ctre::TalonFX& esc_;

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

}  // namespace frcLib846::motor

#endif  // frcLib846_MOTOR_HELPER_H_