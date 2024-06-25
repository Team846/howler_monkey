#ifndef FRC846_ESC_CONTROL_H_
#define FRC846_ESC_CONTROL_H_

#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <units/current.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <initializer_list>
#include <variant>

#include "frc846/control/controlgains.h"
#include "frc846/ctre_namespace.h"
#include "frc846/loggable.h"
#include "frc846/util/conversions.h"
#include "qwertyctre.h"
#include "qwertyrev.h"

FRC846_CTRE_NAMESPACE()

namespace frc846::control {

static constexpr units::millisecond_t CANTimeout = 50_ms;

enum ControlMode { Percent, Velocity, Position, Current };
enum IdleMode { kCoast, kBrake };

constexpr rev::CANSparkMax::ControlType LocalREVControlMode(ControlMode mode) {
  switch (mode) {
    case ControlMode::Percent:
      return rev::CANSparkBase::ControlType::kDutyCycle;
    case ControlMode::Velocity:
      return rev::CANSparkBase::ControlType::kVelocity;
    case ControlMode::Position:
      return rev::CANSparkBase::ControlType::kPosition;
    case ControlMode::Current:
      return rev::CANSparkBase::ControlType::kCurrent;
    default:
      throw std::runtime_error("unsupported control type");
  }
}

template <typename X>
class ElectronicSpeedController {
  using V = units::unit_t<units::compound_unit<typename X::unit_type,
                                               units::inverse<units::second>>>;

  using A = units::ampere_t;

 public:
  virtual int Setup(units::millisecond_t timeout,
                    ControlGainsHelper* gainsHelper, bool isInverted = false,
                    IdleMode kIdleMode = IdleMode::kCoast) = 0;

  virtual int Setup(ControlGainsHelper* gainsHelper, bool isInverted = false,
                    IdleMode kIdleMode = kCoast) = 0;

  virtual int Reset(units::millisecond_t timeout) = 0;

  virtual void SetupConverter(X conv) = 0;

  virtual void ZeroEncoder(X pos) = 0;

  virtual void ZeroEncoder() = 0;

  virtual void Write(ControlMode mode, double output) = 0;

  virtual void Write(ControlMode mode, V output) = 0;

  virtual void Write(ControlMode mode, X output) = 0;

  virtual V GetVelocity() = 0;

  virtual X GetPosition() = 0;

  virtual bool VerifyConnected() = 0;

  virtual bool GetInverted() = 0;

  virtual void SetInverted(bool invert) = 0;

  virtual units::ampere_t GetCurrent() = 0;
};

template <typename X>
class SparkRevController : ElectronicSpeedController<X> {
  using V = units::unit_t<units::compound_unit<typename X::unit_type,
                                               units::inverse<units::second>>>;

  using A = units::ampere_t;

 public:
  SparkRevController(Loggable parent_, std::string name, int kCANid,
                     rev::CANSparkLowLevel::MotorType type =
                         rev::CANSparkLowLevel::MotorType::kBrushless)
      : obj(parent_, name),
        esc_{kCANid, type},
        encoder_{
            esc_.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)},
        pid_controller_{esc_.GetPIDController()} {};

  ~SparkRevController() {
    if (gains_helper != nullptr) {
      delete gains_helper;
    }
  };

  ElectronicSpeedController<X>* that() { return this; }

  int Setup(units::millisecond_t timeout, ControlGainsHelper* gainsHelper,
            bool isInverted = false, IdleMode kIdleMode = IdleMode::kBrake) {
    if (VerifyConnected()) {
      setup = true;
    } else {
      obj.Error("? controller could NOT be setup");
      return -1;
    }

    if (kIdleMode == kCoast)
      CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast),
              "Coast Mode");
    else if (kIdleMode == kBrake)
      CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake),
              "Brake Mode");

    CheckOk(obj, esc_.SetCANTimeout(timeout.to<double>()), "CAN Timeout");
    esc_.SetInverted(isInverted);
    gains_helper = gainsHelper;

    if (gains_helper != nullptr) {
      gains_helper->Write(pid_controller_, gains_cache_, true);
      pid_controller_.SetOutputRange(-gains_helper->peak_output_.value(),
                                     gains_helper->peak_output_.value());
    }

    // esc_.SetOpenLoopRampRate(1.0);
    // esc_.SetClosedLoopRampRate(1.0);

    CheckOk(obj, esc_.SetSmartCurrentLimit(gainsHelper->current_limit_.value()),
            "Current Limit");
    CheckOk(obj, esc_.EnableVoltageCompensation(12.0), "Voltage Compensation");

    CheckOk(obj, esc_.BurnFlash(), "Burn Flash");

    esc_.SetPeriodicFramePeriod(rev::CANSparkBase::PeriodicFrame::kStatus0, 30);

    return 0;
  };

  int Setup(ControlGainsHelper* gainsHelper, bool isInverted = false,
            IdleMode kIdleMode = IdleMode::kBrake) {
    if (VerifyConnected()) {
      setup = true;
    } else {
      obj.Error("? controller could NOT be setup");
      return -1;
    }

    if (kIdleMode == kCoast)
      CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast),
              "Coast Mode");
    else if (kIdleMode == kBrake)
      CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake),
              "Brake Mode");
    CheckOk(obj, esc_.SetCANTimeout(CANTimeout.to<double>()), "CAN Timeout");
    esc_.SetInverted(isInverted);
    gains_helper = gainsHelper;

    if (gains_helper != nullptr) {
      gains_helper->Write(pid_controller_, gains_cache_, true);
      pid_controller_.SetOutputRange(-gains_helper->peak_output_.value(),
                                     gains_helper->peak_output_.value());
    }

    // esc_.SetOpenLoopRampRate(1.0);
    // esc_.SetClosedLoopRampRate(1.0);

    CheckOk(obj, esc_.SetSmartCurrentLimit(gainsHelper->current_limit_.value()),
            "Current Limit");
    CheckOk(obj, esc_.EnableVoltageCompensation(12.0), "Voltage Compensation");

    CheckOk(obj, esc_.BurnFlash(), "Burn Flash");

    esc_.SetPeriodicFramePeriod(rev::CANSparkBase::PeriodicFrame::kStatus0, 30);

    return 0;
  };

  int Reset(units::millisecond_t timeout) {
    if (!setup) return -1;

    CheckOk(obj, esc_.SetCANTimeout(timeout.to<double>()), "CAN Timeout");

    if (gains_helper != nullptr) {
      gains_helper->Write(pid_controller_, gains_cache_, true);
      pid_controller_.SetOutputRange(-gains_helper->peak_output_.value(),
                                     gains_helper->peak_output_.value());
      CheckOk(obj,
              esc_.SetSmartCurrentLimit(gains_helper->current_limit_.value()),
              "Current Limit");
    }

    // esc_.SetOpenLoopRampRate(1.0);
    // esc_.SetClosedLoopRampRate(1.0);

    CheckOk(obj, esc_.EnableVoltageCompensation(12.0), "Voltage Compensation");

    CheckOk(obj, esc_.BurnFlash(), "Burn Flash");

    esc_.SetPeriodicFramePeriod(rev::CANSparkBase::PeriodicFrame::kStatus0, 30);

    return 0;
  };

  void DisableStatusFrames(
      std::initializer_list<rev::CANSparkLowLevel::PeriodicFrame> frames) {
    if (!setup) return;

    for (auto f : frames) {
      // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
      auto err = esc_.SetPeriodicFramePeriod(f, 65535);
      CheckOk(obj, err, "Disable Status Frame");
    }
  };

  void SetupConverter(X conv) {
    if (!setup) return;

    conv_.ChangeConversion(conv);
  };

  void ZeroEncoder(X pos) {
    if (!setup) return;

    CheckOk(obj, encoder_.SetPosition(conv_.RealToNativePosition(pos)),
            "Zero Encoder");
  };

  void ZeroEncoder() {
    if (!setup) return;

    CheckOk(obj, encoder_.SetPosition(0.0), "Zero Encoder");
  };

  void Write(ControlMode mode, double output) {
    if (!setup) return;

    if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
      Reset(CANTimeout);
      CheckOk(obj, esc_.ClearFaults(), "Clear Faults Post Reset");
    }

    if (mode == ControlMode::Percent) {
      auto peak_output = gains_helper->peak_output_.value();

      double value =
          std::max(output, gains_helper->reverse_peak_output_.value());
      value = std::min(value, +peak_output);

      CheckOk(obj,
              pid_controller_.SetReference(value, LocalREVControlMode(mode)),
              "Write Duty Cycle");
    }
  };

  void Write(ControlMode mode, V output) {
    if (!setup) return;

    if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
      Reset(CANTimeout);
      CheckOk(obj, esc_.ClearFaults(), "Clear Faults Post Reset");
    }

    if (gains_helper != nullptr) {
      gains_helper->Write(pid_controller_, gains_cache_, false);
    }

    if (mode == ControlMode::Velocity) {
      CheckOk(obj,
              pid_controller_.SetReference(conv_.RealToNativeVelocity(output),
                                           LocalREVControlMode(mode)),
              "Write Velocity");
    }
  };

  void Write(ControlMode mode, X output) {
    if (!setup) return;

    if (usingPositionLimits) {
      output = units::math::max(output, reverse_position_limit);
      output = units::math::min(output, forward_position_limit);
    }

    if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
      Reset(CANTimeout);
      CheckOk(obj, esc_.ClearFaults(), "Clear Faults Post Reset");
    }

    if (gains_helper != nullptr) {
      gains_helper->Write(pid_controller_, gains_cache_, false);
    }

    if (mode == ControlMode::Position) {
      CheckOk(obj,
              pid_controller_.SetReference(conv_.RealToNativePosition(output),
                                           LocalREVControlMode(mode)),
              "Write Position");
    }
  };

  void Write(ControlMode mode, A output) {
    if (!setup) return;
    if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
      Reset(CANTimeout);
      CheckOk(obj, esc_.ClearFaults(), "Clear Faults Post Reset");
    }

    if (mode == ControlMode::Current) {
      CheckOk(obj,
              pid_controller_.SetReference(output.to<double>(),
                                           LocalREVControlMode(mode)),
              "Write Current");
    }
  };

  void WriteByCurrent(units::ampere_t current) {
    if (!setup) return;

    CheckOk(
        obj,
        pid_controller_.SetReference(current.to<double>(),
                                     LocalREVControlMode(ControlMode::Current)),
        "Write Current");
  }

  V GetVelocity() {
    if (!setup) return units::make_unit<V>(0);

    return conv_.NativeToRealVelocity(encoder_.GetVelocity());
  };

  X GetPosition() {
    if (!setup) return units::make_unit<X>(0);

    return conv_.NativeToRealPosition(encoder_.GetPosition());
  };

  bool VerifyConnected() {
    // GetFirmwareVersion sometimes returns 0 the first time you call it
    esc_.GetFirmwareVersion();
    return esc_.GetFirmwareVersion() != 0;
  };

  bool GetInverted() {
    if (!setup) return false;

    esc_.GetInverted();
    return esc_.GetInverted();
  };

  void SetInverted(bool invert) {
    if (!setup) return;

    esc_.SetInverted(invert);
  };

  units::ampere_t GetCurrent() {
    if (!setup) return 0_A;

    return units::ampere_t(esc_.GetOutputCurrent());
  };

  void EnablePositionLimiting(bool enable = true) {
    usingPositionLimits = enable;
  }

  void ConfigurePositionLimits(X forward_limit, X reverse_limit,
                               bool enable = true) {
    forward_position_limit = forward_limit;
    reverse_position_limit = reverse_limit;
    EnablePositionLimiting(enable);
  }

  Loggable obj;
  rev::CANSparkMax esc_;

 private:
  ControlGainsHelper* gains_helper;

  ControlGains gains_cache_;

  rev::SparkRelativeEncoder encoder_;

  rev::SparkPIDController pid_controller_;

  util::Converter<X> conv_{util::kSparkMAXPeriod, util::kSparkMAXSensorTicks,
                           units::make_unit<X>(1.0)};

  bool setup = false;

  bool usingPositionLimits = false;

  X reverse_position_limit = units::make_unit<X>(0.0);
  X forward_position_limit = units::make_unit<X>(0.0);

  void CheckOk(Loggable& loggable, rev::REVLibError err,
               std::string field = "?") {
    if (err != rev::REVLibError::kOk) {
      loggable.Warn("Unable to update {}", field);
    }
  };
};

template <typename X>
class SparkFlexController : ElectronicSpeedController<X> {
  using V = units::unit_t<units::compound_unit<typename X::unit_type,
                                               units::inverse<units::second>>>;

  using A = units::ampere_t;

 public:
  SparkFlexController(Loggable parent_, std::string name, int kCANid,
                      rev::CANSparkLowLevel::MotorType type =
                          rev::CANSparkLowLevel::MotorType::kBrushless)
      : obj(parent_, name),
        esc_{kCANid, type},
        encoder_{
            esc_.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)},
        pid_controller_{esc_.GetPIDController()} {};

  ~SparkFlexController() {
    if (gains_helper != nullptr) {
      delete gains_helper;
    }
  };

  ElectronicSpeedController<X>* that() { return this; }

  int Setup(units::millisecond_t timeout, ControlGainsHelper* gainsHelper,
            bool isInverted = false, IdleMode kIdleMode = IdleMode::kBrake) {
    if (VerifyConnected()) {
      setup = true;
    } else {
      obj.Error("? controller could NOT be setup");
      return -1;
    }

    if (kIdleMode == kCoast)
      CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast),
              "Coast Mode");
    else if (kIdleMode == kBrake)
      CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake),
              "Brake Mode");

    esc_.SetCANTimeout(timeout.to<double>());
    esc_.SetInverted(isInverted);
    gains_helper = gainsHelper;

    if (gains_helper != nullptr) {
      gains_helper->Write(pid_controller_, gains_cache_, true);
      pid_controller_.SetOutputRange(gains_helper->reverse_peak_output_.value(),
                                     gains_helper->peak_output_.value());
    }

    esc_.SetSmartCurrentLimit(gainsHelper->current_limit_.value());
    esc_.EnableVoltageCompensation(12.0);

    esc_.SetClosedLoopRampRate(gains_helper->ramp_rate_.value());
    esc_.SetOpenLoopRampRate(gains_helper->ramp_rate_.value());

    esc_.BurnFlash();

    esc_.SetPeriodicFramePeriod(rev::CANSparkBase::PeriodicFrame::kStatus0, 30);

    return 0;
  };

  int Setup(ControlGainsHelper* gainsHelper, bool isInverted = false,
            IdleMode kIdleMode = IdleMode::kBrake) {
    if (VerifyConnected()) {
      setup = true;
    } else {
      obj.Error("? controller could NOT be setup");
      return -1;
    }

    if (kIdleMode == kCoast)
      CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast),
              "Coast Mode");
    else if (kIdleMode == kBrake)
      CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake),
              "Brake Mode");

    esc_.SetCANTimeout(CANTimeout.to<double>());
    esc_.SetInverted(isInverted);
    gains_helper = gainsHelper;

    if (gains_helper != nullptr) {
      gains_helper->Write(pid_controller_, gains_cache_, true);
      pid_controller_.SetOutputRange(gains_helper->reverse_peak_output_.value(),
                                     gains_helper->peak_output_.value());
    }

    esc_.SetSmartCurrentLimit(gainsHelper->current_limit_.value());
    esc_.EnableVoltageCompensation(12.0);

    esc_.SetClosedLoopRampRate(gains_helper->ramp_rate_.value());
    esc_.SetOpenLoopRampRate(gains_helper->ramp_rate_.value());

    esc_.BurnFlash();

    esc_.SetPeriodicFramePeriod(rev::CANSparkBase::PeriodicFrame::kStatus0, 30);

    return 0;
  };

  int Reset(units::millisecond_t timeout) {
    if (!setup) return -1;

    esc_.SetCANTimeout(timeout.to<double>());

    if (gains_helper != nullptr) {
      gains_helper->Write(pid_controller_, gains_cache_, true);
      pid_controller_.SetOutputRange(gains_helper->reverse_peak_output_.value(),
                                     gains_helper->peak_output_.value());
      esc_.SetSmartCurrentLimit(gains_helper->current_limit_.value());
    }

    esc_.EnableVoltageCompensation(12.0);

    esc_.SetClosedLoopRampRate(gains_helper->ramp_rate_.value());
    esc_.SetOpenLoopRampRate(gains_helper->ramp_rate_.value());

    esc_.BurnFlash();

    esc_.SetPeriodicFramePeriod(rev::CANSparkBase::PeriodicFrame::kStatus0, 30);

    return 0;
  };

  void DisableStatusFrames(
      std::initializer_list<rev::CANSparkLowLevel::PeriodicFrame> frames) {
    if (!setup) return;

    for (auto f : frames) {
      // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
      auto err = esc_.SetPeriodicFramePeriod(f, 65535);
      CheckOk(obj, err, "Disable Status Frame");
    }
  };

  void SetupConverter(X conv) {
    if (!setup) return;

    conv_.ChangeConversion(conv);
  };

  void ZeroEncoder(X pos) {
    if (!setup) return;

    encoder_.SetPosition(conv_.RealToNativePosition(pos));
  };

  void ZeroEncoder() {
    if (!setup) return;

    CheckOk(obj, encoder_.SetPosition(0.0), "Zero Encoder");
  };

  void Write(ControlMode mode, double output) {
    if (!setup) return;

    if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
      Reset(CANTimeout);
      esc_.ClearFaults();
    }

    if (mode == ControlMode::Percent) {
      auto peak_output = gains_helper->peak_output_.value();

      double value =
          std::max(output, gains_helper->reverse_peak_output_.value());
      value = std::min(value, +peak_output);

      pid_controller_.SetReference(value, LocalREVControlMode(mode));
    }
  };

  void Write(ControlMode mode, V output) {
    if (!setup) return;

    if (esc_.GetStickyFault(rev::CANSparkFlex::FaultID::kHasReset)) {
      Reset(CANTimeout);
      esc_.ClearFaults();
    }

    if (gains_helper != nullptr) {
      gains_helper->Write(pid_controller_, gains_cache_, false);
    }

    if (mode == ControlMode::Velocity) {
      pid_controller_.SetReference(conv_.RealToNativeVelocity(output),
                                   LocalREVControlMode(mode));
    }
  };

  void Write(ControlMode mode, X output) {
    if (!setup) return;

    if (usingPositionLimits) {
      if (output < reverse_position_limit) {
        pid_controller_.SetReference(0.0,
                                     LocalREVControlMode(ControlMode::Percent));
        return;
      } else if (output > forward_position_limit) {
        pid_controller_.SetReference(0.0,
                                     LocalREVControlMode(ControlMode::Percent));
        return;
      }
    }

    if (esc_.GetStickyFault(rev::CANSparkFlex::FaultID::kHasReset)) {
      Reset(CANTimeout);
      esc_.ClearFaults();
    }

    if (gains_helper != nullptr) {
      gains_helper->Write(pid_controller_, gains_cache_, false);
    }

    if (mode == ControlMode::Position) {
      pid_controller_.SetReference(conv_.RealToNativePosition(output),
                                   LocalREVControlMode(mode));
    }
  };

  void Write(ControlMode mode, A output) {
    if (!setup) return;
    if (esc_.GetStickyFault(rev::CANSparkFlex::FaultID::kHasReset)) {
      Reset(CANTimeout);
      CheckOk(obj, esc_.ClearFaults(), "Clear Faults Post Reset");
    }

    if (mode == ControlMode::Current) {
      CheckOk(obj,
              pid_controller_.SetReference(output.to<double>(),
                                           LocalREVControlMode(mode)),
              "Write Current");
    }
  };

  void WriteByCurrent(units::ampere_t current) {
    if (!setup) return;

    CheckOk(
        obj,
        pid_controller_.SetReference(current.to<double>(),
                                     LocalREVControlMode(ControlMode::Current)),
        "Write Current");
  }

  V GetVelocity() {
    if (!setup) return units::make_unit<V>(0);

    return conv_.NativeToRealVelocity(encoder_.GetVelocity());
  };

  X GetPosition() {
    if (!setup) return units::make_unit<X>(0);

    return conv_.NativeToRealPosition(encoder_.GetPosition());
  };

  bool VerifyConnected() {
    // GetFirmwareVersion sometimes returns 0 the first time you call it
    esc_.GetFirmwareVersion();
    return esc_.GetFirmwareVersion() != 0;
  };

  bool GetInverted() {
    if (!setup) return false;

    esc_.GetInverted();
    return esc_.GetInverted();
  };

  void SetInverted(bool invert) {
    if (!setup) return;

    esc_.SetInverted(invert);
  };

  units::ampere_t GetCurrent() {
    if (!setup) return 0_A;

    return units::ampere_t(esc_.GetOutputCurrent());
  };

  void EnablePositionLimiting(bool enable = true) {
    usingPositionLimits = enable;
  }

  void ConfigurePositionLimits(X forward_limit, X reverse_limit,
                               bool enable = true) {
    forward_position_limit = forward_limit;
    reverse_position_limit = reverse_limit;
    EnablePositionLimiting(enable);
  }

  Loggable obj;
  rev::CANSparkFlex esc_;

 private:
  ControlGainsHelper* gains_helper;

  ControlGains gains_cache_;

  rev::SparkRelativeEncoder encoder_;

  rev::SparkPIDController pid_controller_;

  util::Converter<X> conv_{util::kSparkMAXPeriod, util::kSparkMAXSensorTicks,
                           units::make_unit<X>(1.0)};

  bool setup = false;

  bool usingPositionLimits = false;

  X reverse_position_limit = units::make_unit<X>(0.0);
  X forward_position_limit = units::make_unit<X>(0.0);

  void CheckOk(Loggable& loggable, rev::REVLibError err,
               std::string field = "?") {
    if (err != rev::REVLibError::kOk) {
      loggable.Warn("Unable to update {}", field);
    }
  };
};

/*
 *
 *
 *
 */

template <typename X>
class TalonFXController : ElectronicSpeedController<X> {
  using V = units::unit_t<units::compound_unit<typename X::unit_type,
                                               units::inverse<units::second>>>;

  using A = units::ampere_t;

 public:
  TalonFXController(Loggable parent_, std::string name, int kCanIdPort)
      : obj(parent_, name),
        esc_{kCanIdPort},
        configurator_{esc_.GetConfigurator()} {};

  ~TalonFXController() {
    if (gains_helper != nullptr) {
      delete gains_helper;
    }
  };

  ElectronicSpeedController<X>* that() { return this; }

  int Setup(units::millisecond_t timeout, ControlGainsHelper* gainsHelper,
            bool isInverted = false, IdleMode kIdleMode = kCoast) {
    if (VerifyConnected()) {
      setup = true;
    } else {
      obj.Error("? controller could NOT be setup");
      return -1;
    }

    gains_helper = gainsHelper;

    ctre::configs::VoltageConfigs voltageConfs{};
    ctre::configs::CurrentLimitsConfigs currentConfs{};
    ctre::configs::MotorOutputConfigs motorConfs{};

    motorConfs.WithInverted(isInverted);

    if (kIdleMode == kCoast)
      motorConfs.WithNeutralMode(
          ctre::phoenix6::signals::NeutralModeValue::Coast);
    else if (kIdleMode == kBrake)
      motorConfs.WithNeutralMode(
          ctre::phoenix6::signals::NeutralModeValue::Brake);

    deviceConfigs.WithMotorOutput(motorConfs);

    currentConfs.WithSupplyCurrentLimitEnable(true);
    currentConfs.WithSupplyCurrentLimit(gains_helper->current_limit_.value());
    currentConfs.WithSupplyCurrentThreshold(
        gains_helper->current_limit_.value() * 1.5);
    currentConfs.WithSupplyTimeThreshold(0.20);

    voltageConfs.PeakForwardVoltage = 16.0;
    voltageConfs.PeakReverseVoltage = -16.0;

    deviceConfigs.WithVoltage(voltageConfs);
    deviceConfigs.WithCurrentLimits(currentConfs);

    CheckOk(obj, configurator_.Apply(deviceConfigs));

    if (gains_helper != nullptr) {
      gains_helper->Write(configurator_, deviceConfigs, gains_cache_, false);
    }

    esc_.OptimizeBusUtilization();

    esc_.GetPosition().SetUpdateFrequency(50_Hz);
    esc_.GetVelocity().SetUpdateFrequency(50_Hz);
    esc_.GetSupplyCurrent().SetUpdateFrequency(10_Hz);
    // esc_.GetSupplyVoltage().SetUpdateFrequency(10_Hz);
    // esc_.GetForwardLimit().SetUpdateFrequency(50_Hz);
    // esc_.GetReverseLimit().SetUpdateFrequency(50_Hz);

    return 0;
  };

  int Setup(ControlGainsHelper* gainsHelper, bool isInverted = false,
            IdleMode kIdleMode = IdleMode::kCoast) {
    if (VerifyConnected()) {
      setup = true;
    } else {
      obj.Error("? controller could NOT be setup");
      return -1;
    }

    gains_helper = gainsHelper;

    ctre::configs::VoltageConfigs voltageConfs{};
    ctre::configs::CurrentLimitsConfigs currentConfs{};
    ctre::configs::MotorOutputConfigs motorConfs{};

    motorConfs.WithInverted(isInverted);

    if (kIdleMode == kCoast)
      motorConfs.WithNeutralMode(
          ctre::phoenix6::signals::NeutralModeValue::Coast);
    else if (kIdleMode == kBrake)
      motorConfs.WithNeutralMode(
          ctre::phoenix6::signals::NeutralModeValue::Brake);

    deviceConfigs.WithMotorOutput(motorConfs);

    currentConfs.WithSupplyCurrentLimitEnable(true);
    currentConfs.WithSupplyCurrentLimit(gains_helper->current_limit_.value());
    currentConfs.WithSupplyCurrentThreshold(
        gains_helper->current_limit_.value() * 1.5);
    currentConfs.WithSupplyTimeThreshold(0.20);

    voltageConfs.PeakForwardVoltage = 16.0;
    voltageConfs.PeakReverseVoltage = -16.0;

    deviceConfigs.WithVoltage(voltageConfs);
    deviceConfigs.WithCurrentLimits(currentConfs);

    CheckOk(obj, configurator_.Apply(deviceConfigs));

    if (gains_helper != nullptr) {
      gains_helper->Write(configurator_, deviceConfigs, gains_cache_, false);
    }

    esc_.OptimizeBusUtilization();

    esc_.GetPosition().SetUpdateFrequency(50_Hz);
    esc_.GetVelocity().SetUpdateFrequency(50_Hz);
    esc_.GetSupplyCurrent().SetUpdateFrequency(10_Hz);
    // esc_.GetSupplyVoltage().SetUpdateFrequency(10_Hz);
    // esc_.GetForwardLimit().SetUpdateFrequency(50_Hz);
    // esc_.GetReverseLimit().SetUpdateFrequency(50_Hz);

    return 0;
  };

  int Reset(units::millisecond_t timeout) {
    if (!setup) return -1;

    ctre::configs::VoltageConfigs voltageConfs{};
    ctre::configs::CurrentLimitsConfigs currentConfs{};

    currentConfs.WithSupplyCurrentLimitEnable(true);
    currentConfs.WithSupplyCurrentLimit(gains_helper->current_limit_.value());
    currentConfs.WithSupplyCurrentThreshold(
        gains_helper->current_limit_.value() * 1.5);
    currentConfs.WithSupplyTimeThreshold(0.20);

    voltageConfs.PeakForwardVoltage = 16.0;
    voltageConfs.PeakReverseVoltage = -16.0;

    deviceConfigs.WithVoltage(voltageConfs);
    deviceConfigs.WithCurrentLimits(currentConfs);

    CheckOk(obj, configurator_.Apply(deviceConfigs));

    if (gains_helper != nullptr) {
      gains_helper->Write(configurator_, deviceConfigs, gains_cache_, false);
    }

    esc_.OptimizeBusUtilization();

    esc_.GetPosition().SetUpdateFrequency(50_Hz);
    esc_.GetVelocity().SetUpdateFrequency(50_Hz);
    esc_.GetSupplyCurrent().SetUpdateFrequency(10_Hz);
    // esc_.GetSupplyVoltage().SetUpdateFrequency(10_Hz);
    // esc_.GetForwardLimit().SetUpdateFrequency(50_Hz);
    // esc_.GetReverseLimit().SetUpdateFrequency(50_Hz);

    return 0;
  };

  void SetupConverter(X conv) {
    if (!setup) return;

    conv_.ChangeConversion(conv);
  };

  void ZeroEncoder(X pos) {
    if (!setup) return;

    esc_.SetPosition(units::turn_t(conv_.RealToNativePosition(pos)));
  };

  void ZeroEncoder() {
    if (!setup) return;

    esc_.SetPosition(0.0_tr);
  };

  void Write(ControlMode mode, double output) {
    if (!setup) return;

    if (esc_.HasResetOccurred()) {
      Reset(CANTimeout);
      esc_.ClearStickyFaults();
    }

    // CheckOk(obj, configurator_.Apply(deviceConfigs));

    if (mode == ControlMode::Percent) {
      auto peak_output = gains_helper->peak_output_.value();

      double value = std::max(output, -peak_output);
      value = std::min(value, +peak_output);

      esc_.Set(value);
    }
  };

  void Write(ControlMode mode, V output) {
    if (!setup) return;

    if (esc_.HasResetOccurred()) {
      Reset(CANTimeout);
      esc_.ClearStickyFaults();
    }

    if (gains_helper != nullptr) {
      gains_helper->Write(configurator_, deviceConfigs, gains_cache_, false);
    }

    // CheckOk(obj, configurator_.Apply(deviceConfigs));
    if (mode == ControlMode::Velocity) {
      ctre::controls::VelocityDutyCycle cntrl{
          units::turns_per_second_t(conv_.RealToNativeVelocity(output))};
      esc_.SetControl(cntrl);
    }
  };

  void Write(ControlMode mode, X output) {
    if (!setup) return;

    if (esc_.HasResetOccurred()) {
      Reset(CANTimeout);
      esc_.ClearStickyFaults();
    }

    if (gains_helper != nullptr) {
      gains_helper->Write(configurator_, deviceConfigs, gains_cache_, false);
    }

    // CheckOk(obj, configurator_.Apply(deviceConfigs));

    if (mode == ControlMode::Position) {
      ctre::controls::PositionDutyCycle cntrl{
          units::turn_t(conv_.RealToNativePosition(output))};
      esc_.SetControl(cntrl);
    }
  };

  V GetVelocity() {
    if (!setup) return units::make_unit<V>(0);

    return conv_.NativeToRealVelocity(esc_.GetVelocity().GetValueAsDouble());
  };

  X GetPosition() {
    if (!setup) return units::make_unit<X>(0);

    return conv_.NativeToRealPosition(esc_.GetPosition().GetValueAsDouble());
  };

  bool VerifyConnected() { return esc_.IsAlive(); };

  bool GetInverted() {
    if (!setup) return false;

    esc_.GetInverted();
    return esc_.GetInverted();
  };

  void SetInverted(bool invert) {
    if (!setup) return;

    ctre::configs::MotorOutputConfigs motorConfs{};
    ctre::configs::Slot0Configs pidConfs{};

    motorConfs.WithInverted(invert);

    deviceConfigs.WithMotorOutput(motorConfs);

    CheckOk(obj, configurator_.Apply(deviceConfigs));
  };

  units::ampere_t GetCurrent() {
    if (!setup) return 0_A;

    return 0_A;
  };

 private:
  Loggable obj;
  ControlGainsHelper* gains_helper;

  ctre::phoenix6::hardware::TalonFX esc_;
  ctre::phoenix6::configs::TalonFXConfigurator& configurator_;

  ctre::phoenix6::configs::TalonFXConfiguration deviceConfigs{};

  ControlGains gains_cache_;

  util::Converter<X> conv_{util::kTalonPeriod, util::kTalonFXSensorTicks,
                           units::make_unit<X>(1.0)};

  bool setup = false;

  void CheckOk(Loggable& loggable, ctre::phoenix::StatusCode err,
               std::string field = "TalonFX Config") {
    if (!(err.IsOK() || err.IsWarning())) {
      loggable.Warn("Unable to update {}", field);
    }
  };
};

};  // namespace frc846::control

#endif