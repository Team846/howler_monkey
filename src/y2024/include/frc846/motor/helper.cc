#include "frc846/motor/helper.h"
#include <ctre/phoenix6/TalonFX.hpp>

namespace frc846::motor {

constexpr rev::CANSparkMax::ControlType RevControlMode(ControlMode mode) {
  switch (mode) {
    case ControlMode::Percent:
      return rev::CANSparkMax::ControlType::kDutyCycle;
    case ControlMode::Velocity:
      return rev::CANSparkMax::ControlType::kVelocity;
    case ControlMode::Position:
      return rev::CANSparkMax::ControlType::kPosition;
    case ControlMode::Current:
      return rev::CANSparkMax::ControlType::kCurrent;
    default:
      throw std::runtime_error("unsupported control type");
  }
}

void CheckOk(Loggable loggable, auto err, std::string_view field) {
  if (err.IsAllGood()) {
    loggable.Error("Unable to update {}", field);
  }
}

void CheckOk(Loggable loggable, rev::REVLibError err, std::string_view field) {
  if (err != rev::REVLibError::kOk) {
    loggable.Error("Unable to update {}", field);
  }
}

SparkMAXHelper::SparkMAXHelper(Loggable parent, rev::CANSparkMax& esc,
                               SparkMAXConfigHelper* config, GainsHelper* gains)
    : parent_(parent),
      esc_(esc),
      pid_controller_(esc.GetPIDController()),
      encoder_(esc.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      config_(config),
      gains_(gains) {}

SparkMAXHelper::~SparkMAXHelper() {
  if (config_ != nullptr) {
    delete config_;
    config_ = nullptr;
  }
  if (gains_ != nullptr) {
    delete gains_;
    gains_ = nullptr;
  }
}

void SparkMAXHelper::Setup(units::millisecond_t timeout) {
  esc_.RestoreFactoryDefaults();
  esc_.SetCANTimeout(timeout.to<double>());
  esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  if (config_ != nullptr) {
    config_->Write(esc_, pid_controller_, config_cache_, true);
  }
  if (gains_ != nullptr) {
    gains_->Write(pid_controller_, gains_cache_, true);
  }

  for (auto c : on_inits_) {
    c();
  }

  esc_.BurnFlash();
}

void SparkMAXHelper::Setup(bool brake, units::millisecond_t timeout) {
  esc_.RestoreFactoryDefaults();
  esc_.SetCANTimeout(timeout.to<double>());
  if (brake) {
    esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  } else {
    esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  }

  if (config_ != nullptr) {
    config_->Write(esc_, pid_controller_, config_cache_, true);
  }
  if (gains_ != nullptr) {
    gains_->Write(pid_controller_, gains_cache_, true);
  }

  for (auto c : on_inits_) {
    c();
  }

  esc_.BurnFlash();
}

void SparkMAXHelper::DisableStatusFrames(
    std::initializer_list<rev::CANSparkLowLevel::PeriodicFrame> frames) {
  for (auto f : frames) {
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
    auto err = esc_.SetPeriodicFramePeriod(f, 65535);
    CheckOk(parent_, err, "status frame");
  }
}

void SparkMAXHelper::OnInit(std::function<void()> callback) {
  on_inits_.push_back(callback);
}

bool SparkMAXHelper::VerifyConnected() {
  // TODO this is stupid...
  // GetFirmwareVersion sometimes returns 0 the first time you call it
  esc_.GetFirmwareVersion();
  return esc_.GetFirmwareVersion() != 0;
}

void SparkMAXHelper::Write(Output output) {
  if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
    parent_.Warn("Reset detected!! Rewriting config...");
    Setup(kCANTimeout);
    esc_.ClearFaults();
  }

  if (config_ != nullptr) {
    config_->Write(esc_, pid_controller_, config_cache_);
  }
  if (gains_ != nullptr) {
    gains_->Write(pid_controller_, gains_cache_);
  }

  auto value = output.value;

  // Clamp output value to peak output range if using open loop.
  //
  // Configuring the output range of the PIDController in SparkMAXConfigHelper
  // only affects closed loop control.
  if (output.mode == ControlMode::Percent) {
    auto peak_output = config_->peak_output_.value();

    value = std::max(value, -peak_output);
    value = std::min(value, +peak_output);
  }

  pid_controller_.SetReference(value, RevControlMode(output.mode));
}

}  // namespace frc846::motor