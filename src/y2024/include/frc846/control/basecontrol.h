#ifndef FRC846_BASE_ESC_CONTROL_H_
#define FRC846_BASE_ESC_CONTROL_H_

#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <units/current.h>

#include <initializer_list>
#include <variant>

#include "frc846/control/controlgains.h"
#include "frc846/ctre_namespace.h"
#include "frc846/loggable.h"
#include "frc846/util/conversions.h"

FRC846_CTRE_NAMESPACE()

namespace frc846::control {

static constexpr units::millisecond_t CANTimeout = 50_ms;

constexpr unsigned int errorParseREVLib(rev::REVLibError err) {
  switch (err) {
    case rev::REVLibError::kOk:
      return 0;
      break;
    case rev::REVLibError::kError:
      return 1;
      break;
    case rev::REVLibError::kTimeout:
      return 2;
      break;
    case rev::REVLibError::kHALError:
      return 3;
      break;
    case rev::REVLibError::kCantFindFirmware:
      return 4;
      break;
    case rev::REVLibError::kFirmwareTooNew:
      return 5;
      break;
    case rev::REVLibError::kFirmwareTooOld:
      return 6;
      break;
    case rev::REVLibError::kInvalidCANId:
      return 7;
      break;
    case rev::REVLibError::kDuplicateCANId:
      return 8;
      break;
    case rev::REVLibError::kFollowConfigMismatch:
      return 9;
      break;
    case rev::REVLibError::kCANDisconnected:
      return 10;
    default:
      return -1;
      break;
  }
}


constexpr std::string parseControllerError(unsigned int err) {
  switch (err) {
    case 0:
      return "OK";
      break;
    case 1:
      return "Unspecified error";
      break;
    case 2:
      return "Timeout";
      break;
    case 3:
      return "HAL error";
      break;
    case 4:
      return "Firmware not found";
      break;
    case 5:
    case 6:
      return "Firmware version mismatch";
      break;
    case 7:
      return "Invalid CAN ID";
      break;
    case 8:
      return "Duplicate CAN ID";
      break;
    case 9:
      return "Follower configuration issue";
      break;
    case 10:
      return "CAN disconnect";
      break;
    case 11:
      return "Unknown warning";
    default:
      return "Unknown error";
      break;
  }
}

class BaseElectronicSpeedController {
 public:
  BaseElectronicSpeedController* that() { return this; };

  virtual unsigned int InitConfigure(units::millisecond_t timeout) = 0;

  virtual unsigned int AddGainsHelper(ControlGainsHelper* gainsHelper) = 0;

  virtual unsigned int ZeroEncoder(double pos) = 0;

  virtual unsigned int WriteDutyCycle(double output) = 0;

  virtual unsigned int WritePosition(double output) = 0;

  virtual unsigned int WriteVelocity(double output) = 0;

  virtual double GetVelocity() = 0;

  virtual double GetPosition() = 0;

  virtual double GetCurrent() = 0;

  virtual bool VerifyConnected() = 0;

  virtual bool GetInverted() = 0;

  virtual unsigned int SetInverted(bool invert) = 0;

  virtual unsigned int SetCoastMode(bool brake) = 0;

  virtual unsigned int SetLimits(double reverse, double forward) = 0;
};

template <typename REVType>
class BaseSparkREVController : BaseElectronicSpeedController {
 public:
  BaseSparkREVController(int kCANid,
                         rev::CANSparkLowLevel::MotorType type =
                             rev::CANSparkLowLevel::MotorType::kBrushless)
      : esc_{kCANid, type},
        encoder_{
            esc_.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)},
        pid_controller_{esc_.GetPIDController()} {
    static_assert(
        std::is_base_of_v<rev::CANSparkBase, REVType>,
        "SparkREVController's type does not inherit from CANSparkBase");
  };

  ~BaseSparkREVController(){};

  unsigned int InitConfigure(units::millisecond_t timeout = 0.0_ms) override {
    if (!VerifyConnected()) return -1;

    rev::REVLibError err = esc_.SetCANTimeout(timeout.to<double>());
    if (err != rev::REVLibError::kOk) return errorParseREVLib(err);
    err = esc_.SetSmartCurrentLimit(40.0);
    if (err != rev::REVLibError::kOk) return errorParseREVLib(err);
    err = esc_.EnableVoltageCompensation(12.0);
    if (err != rev::REVLibError::kOk) return errorParseREVLib(err);
    err = esc_.SetPeriodicFramePeriod(
        rev::CANSparkBase::PeriodicFrame::kStatus0, 30);
    if (err != rev::REVLibError::kOk) return errorParseREVLib(err);
    err = esc_.BurnFlash();
    if (err != rev::REVLibError::kOk) return errorParseREVLib(err);

    setup = true;

    return errorParseREVLib(rev::REVLibError::kOk);
  }

  unsigned int AddGainsHelper(ControlGainsHelper* gainsHelper) override {
    gains_helper = gainsHelper;
    rev::REVLibError err =
        pid_controller_.SetOutputRange(-gains_helper->peak_output_.value(),
                                       gains_helper->peak_output_.value());
    if (err != rev::REVLibError::kOk) return errorParseREVLib(err);
    err = gains_helper->Write(pid_controller_, gains_cache_, true);
    if (err != rev::REVLibError::kOk) return errorParseREVLib(err);

    return errorParseREVLib(rev::REVLibError::kOk);
  }

  unsigned int DisableStatusFrames(
      std::initializer_list<rev::CANSparkLowLevel::PeriodicFrame> frames) {
    if (!setup) return -1;

    for (auto f : frames) {
      // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
      auto err = esc_.SetPeriodicFramePeriod(f, 65535);
      if (err != rev::REVLibError::kOk) {
        return errorParseREVLib(err);
      }
    }

    return errorParseREVLib(rev::REVLibError::kOk);
  };

  unsigned int ZeroEncoder(double pos) {
    if (!setup) return -1;

    return errorParseREVLib(encoder_.SetPosition(pos));
  };

  unsigned int WriteDutyCycle(double output) override {
    if (!setup) return -1;

    if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
      setup = false;
      return -1;
    }

    double value = output;

    if (gainsHelper != nullptr) {
      value = std::max(output, gains_helper->reverse_peak_output_.value());
      value = std::min(value, gains_helper->peak_output_.value());
    }

    return errorParseREVLib(pid_controller_.SetReference(
        value, rev::CANSparkLowLevel::ControlType::kDutyCycle));
  };

  unsigned int WriteVelocity(double output) override {
    if (!setup || gains_helper == nullptr) return -1;

    if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
      setup = false;
      return -1;
    }

    err = gains_helper->Write(pid_controller_, gains_cache_, false);
    if (err != rev::REVLibError::kOk) return errorParseREVLib(err);

    return errorParseREVLib(pid_controller_.SetReference(
        output, rev::CANSparkLowLevel::ControlType::kVelocity));
  };

  unsigned int WritePosition(double output) override {
    if (!setup || gainsHelper == nullptr) return -1;

    if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
      setup = false;
      return -1;
    }

    err = gains_helper->Write(pid_controller_, gains_cache_, false);
    if (err != rev::REVLibError::kOk) return errorParseREVLib(err);

    return errorParseREVLib(pid_controller_.SetReference(
        output, rev::CANSparkLowLevel::ControlType::kPosition));
  };

  double GetVelocity() override {
    if (!setup) return 0.0;

    return encoder_.GetVelocity();
  };

  double GetPosition() override {
    if (!setup) return 0.0;

    return encoder_.GetPosition();
  };

  double GetCurrent() override {
    if (!setup) return 0.0;

    return esc_.GetOutputCurrent();
  };

  bool VerifyConnected() override {
    // GetFirmwareVersion sometimes returns 0 the first time you call it
    esc_.GetFirmwareVersion();
    return esc_.GetFirmwareVersion() != 0;
  };

  bool GetInverted() override {
    if (!setup) return false;

    esc_.GetInverted();
    return esc_.GetInverted();
  };

  unsigned int SetInverted(bool invert) override {
    if (!setup) return -1;

    esc_.SetInverted(invert);
    return errorParseREVLib(rev::REVLibError::kOk);
  };

  unsigned int SetCoastMode(bool brake) override {
    if (!setup) return -1;

    return errorParseREVLib(
        esc_.SetIdleMode(brake ? rev::CANSparkBase::IdleMode::kBrake
                               : rev::CANSparkBase::IdleMode::kCoast));
  };

  unsigned int SetLimits(double reverse, double forward) override {
    if (!setup) return -1;

    rev::REVLibError err = esc_.SetSoftLimit(
        rev::CANSparkBase::SoftLimitDirection::kReverse, reverse);
    if (err != rev::REVLibError::kOk) return errorParseREVLib(err);

    err = esc_.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward,
                            forward);
    return errorParseREVLib(err);
  }

  unsigned int SetLimits(double reverse, double forward) override {
    if (!setup) return -1;

    esc_.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, reverse);
    esc_.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, forward);

    return errorParseREVLib(rev::REVLibError::kOk);
  }

  unsigned int ConfigureAsFollower(rev::CANSparkBase& leader, bool invert) {
    return errorParseREVLib(esc_.Follow(leader, invert));
  }

  rev::SparkLimitSwitch GetReverseHardLimit(rev::SparkLimitSwitch::Type type) {
    return esc_.GetReverseLimitSwitch(type);
  }

  rev::SparkLimitSwitch GetForwardHardLimit(rev::SparkLimitSwitch::Type type) {
    return esc_.GetForwardLimitSwitch(type);
  }

 private:
  REVType esc_;

  ControlGainsHelper* gains_helper;

  ControlGains gains_cache_;

  rev::SparkRelativeEncoder encoder_;

  rev::SparkPIDController pid_controller_;

  bool setup = false;
};

/*
 *
 *
 *
 *
 *
 *
 *
 */

// class BaseTalonFXController : BaseElectronicSpeedController {
//  public:
//   BaseTalonFXController(int kCANid)
//       : esc_{kCANid}, configurator_{esc_.GetConfigurator()} {};

//   ~BaseTalonFXController(){};

//   unsigned int InitConfigure(units::millisecond_t timeout = 0.0_ms) override {
//     if (!VerifyConnected()) return -1;

//     rev::REVLibError err = esc_.SetCANTimeout(timeout.to<double>());
//     if (err != rev::REVLibError::kOk) return errorParseREVLib(err);
//     err = esc_.SetSmartCurrentLimit(40.0);
//     if (err != rev::REVLibError::kOk) return errorParseREVLib(err);
//     err = esc_.EnableVoltageCompensation(12.0);
//     if (err != rev::REVLibError::kOk) return errorParseREVLib(err);
//     err = esc_.SetPeriodicFramePeriod(
//         rev::CANSparkBase::PeriodicFrame::kStatus0, 30);
//     if (err != rev::REVLibError::kOk) return errorParseREVLib(err);
//     err = esc_.BurnFlash();
//     if (err != rev::REVLibError::kOk) return errorParseREVLib(err);

//     setup = true;

//     return errorParseREVLib(rev::REVLibError::kOk);
//   }

//   unsigned int AddGainsHelper(ControlGainsHelper* gainsHelper) override {
//     gains_helper = gainsHelper;
//     rev::REVLibError err =
//         pid_controller_.SetOutputRange(-gains_helper->peak_output_.value(),
//                                        gains_helper->peak_output_.value());
//     if (err != rev::REVLibError::kOk) return errorParseREVLib(err);
//     err = gains_helper->Write(pid_controller_, gains_cache_, true);
//     if (err != rev::REVLibError::kOk) return errorParseREVLib(err);

//     return errorParseREVLib(rev::REVLibError::kOk);
//   }

//   unsigned int DisableStatusFrames(
//       std::initializer_list<rev::CANSparkLowLevel::PeriodicFrame> frames) {
//     if (!setup) return -1;

//     for (auto f : frames) {
//       // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
//       auto err = esc_.SetPeriodicFramePeriod(f, 65535);
//       if (err != rev::REVLibError::kOk) {
//         return errorParseREVLib(err);
//       }
//     }

//     return errorParseREVLib(rev::REVLibError::kOk);
//   };

//   unsigned int ZeroEncoder(double pos) {
//     if (!setup) return -1;

//     return errorParseREVLib(encoder_.SetPosition(pos));
//   };

//   unsigned int WriteDutyCycle(double output) override {
//     if (!setup) return -1;

//     if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
//       setup = false;
//       return -1;
//     }

//     double value = output;

//     if (gainsHelper != nullptr) {
//       value = std::max(output, gains_helper->reverse_peak_output_.value());
//       value = std::min(value, gains_helper->peak_output_.value());
//     }

//     return errorParseREVLib(pid_controller_.SetReference(
//         value, rev::CANSparkLowLevel::ControlType::kDutyCycle));
//   };

//   unsigned int WriteVelocity(double output) override {
//     if (!setup || gains_helper == nullptr) return -1;

//     if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
//       setup = false;
//       return -1;
//     }

//     err = gains_helper->Write(pid_controller_, gains_cache_, false);
//     if (err != rev::REVLibError::kOk) return errorParseREVLib(err);

//     return errorParseREVLib(pid_controller_.SetReference(
//         output, rev::CANSparkLowLevel::ControlType::kVelocity));
//   };

//   unsigned int WritePosition(double output) override {
//     if (!setup || gainsHelper == nullptr) return -1;

//     if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
//       setup = false;
//       return -1;
//     }

//     err = gains_helper->Write(pid_controller_, gains_cache_, false);
//     if (err != rev::REVLibError::kOk) return errorParseREVLib(err);

//     return errorParseREVLib(pid_controller_.SetReference(
//         output, rev::CANSparkLowLevel::ControlType::kPosition));
//   };

//   double GetVelocity() override {
//     if (!setup) return 0.0;

//     return esc_.GetVelocity().GetValueAsDouble();
//   };

//   double GetPosition() override {
//     if (!setup) return 0.0;

//     return esc_.GetPosition().GetValueAsDouble();
//   };

//   double GetCurrent() override {
//     if (!setup) return 0.0;

//     return esc_.GetSupplyCurrent().GetValueAsDouble();
//   };

//   bool VerifyConnected() override { return esc_.IsAlive(); };

//   bool GetInverted() override {
//     if (!setup) return false;

//     return esc_.GetInverted();
//   };

//   unsigned int SetInverted(bool invert) override {
//     if (!setup) return;

//     deviceConfigs.MotorOutput.Inverted = invert;

//     auto status = configurator_.Apply(deviceConfigs);
//     return errorParsePhoenix(status);
//   };

//   unsigned int SetLimits(double reverse, double forward) override {
//     if (!setup) return;

//     deviceConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//     deviceConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
//     deviceConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forward;
//     deviceConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = forward;

//     auto status = configurator_.Apply(deviceConfigs);
//     return errorParsePhoenix(status);
//   }

//  private:
//   ctre::phoenix6::hardware::TalonFX esc_;
//   ctre::phoenix6::configs::TalonFXConfigurator& configurator_;
//   ctre::phoenix6::configs::TalonFXConfiguration deviceConfigs{};

//   ControlGainsHelper* gains_helper;

//   ControlGains gains_cache_;

//   bool setup = false;
// };

// // template <typename X>
// // class TalonFXController : ElectronicSpeedController<X> {
// //   using V = units::unit_t<units::compound_unit<typename X::unit_type,
// // units::inverse<units::second>>>;

// //   using A = units::ampere_t;

// //  public:
// //   TalonFXController(Loggable parent_, std::string name, int kCanIdPort)
// //       : obj(parent_, name),
// //         esc_{kCanIdPort},
// //         configurator_{esc_.GetConfigurator()} {};

// //   ~TalonFXController() {
// //     if (gains_helper != nullptr) {
// //       delete gains_helper;
// //     }
// //   };

// //   ElectronicSpeedController<X>* that() { return this; }

// //   int Setup(units::millisecond_t timeout, ControlGainsHelper* gainsHelper,
// //             bool isInverted = false, IdleMode kIdleMode = kCoast) {
// //     if (VerifyConnected()) {
// //       setup = true;
// //     } else {
// //       obj.Error("? controller could NOT be setup");
// //       return -1;
// //     }

// //     gains_helper = gainsHelper;

// //     ctre::configs::VoltageConfigs voltageConfs{};
// //     ctre::configs::CurrentLimitsConfigs currentConfs{};
// //     ctre::configs::MotorOutputConfigs motorConfs{};

// //     motorConfs.WithInverted(isInverted);

// //     if (kIdleMode == kCoast)
// //       motorConfs.WithNeutralMode(
// //           ctre::phoenix6::signals::NeutralModeValue::Coast);
// //     else if (kIdleMode == kBrake)
// //       motorConfs.WithNeutralMode(
// //           ctre::phoenix6::signals::NeutralModeValue::Brake);

// //     deviceConfigs.WithMotorOutput(motorConfs);

// // currentConfs.WithSupplyCurrentLimit(gains_helper->current_limit_.value()
// //     /
// //                                         2.0);
// //     currentConfs.WithSupplyCurrentThreshold(
// //         gains_helper->current_limit_.value());
// //     currentConfs.WithSupplyTimeThreshold(0.02);
// // currentConfs.WithStatorCurrentLimit(gains_helper->current_limit_.value());
// //     currentConfs.WithStatorCurrentLimitEnable(true);

// //     voltageConfs.PeakForwardVoltage = 12.0;
// //     voltageConfs.PeakReverseVoltage = -12.0;

// //     deviceConfigs.WithVoltage(voltageConfs);
// //     deviceConfigs.WithCurrentLimits(currentConfs);

// //     CheckOk(obj, configurator_.Apply(deviceConfigs));

// //     if (gains_helper != nullptr) {
// //       gains_helper->Write(configurator_, deviceConfigs, gains_cache_,
// false);
// //     }

// //     esc_.OptimizeBusUtilization();

// //     esc_.GetPosition().SetUpdateFrequency(50_Hz);
// //     esc_.GetVelocity().SetUpdateFrequency(50_Hz);
// //     esc_.GetSupplyCurrent().SetUpdateFrequency(10_Hz);
// //     // esc_.GetSupplyVoltage().SetUpdateFrequency(10_Hz);
// //     // esc_.GetForwardLimit().SetUpdateFrequency(50_Hz);
// //     // esc_.GetReverseLimit().SetUpdateFrequency(50_Hz);

// //     return 0;
// //   };

// //   int Setup(ControlGainsHelper* gainsHelper, bool isInverted = false,
// //             IdleMode kIdleMode = IdleMode::kCoast) {
// //     if (VerifyConnected()) {
// //       setup = true;
// //     } else {
// //       obj.Error("? controller could NOT be setup");
// //       return -1;
// //     }

// //     gains_helper = gainsHelper;

// //     ctre::configs::VoltageConfigs voltageConfs{};
// //     ctre::configs::CurrentLimitsConfigs currentConfs{};
// //     ctre::configs::MotorOutputConfigs motorConfs{};

// //     motorConfs.WithInverted(isInverted);

// //     if (kIdleMode == kCoast)
// //       motorConfs.WithNeutralMode(
// //           ctre::phoenix6::signals::NeutralModeValue::Coast);
// //     else if (kIdleMode == kBrake)
// //       motorConfs.WithNeutralMode(
// //           ctre::phoenix6::signals::NeutralModeValue::Brake);

// //     deviceConfigs.WithMotorOutput(motorConfs);

// // currentConfs.WithSupplyCurrentLimit(gains_helper->current_limit_.value()
// //     /
// //                                         2.0);
// //     currentConfs.WithSupplyCurrentThreshold(
// //         gains_helper->current_limit_.value());
// //     currentConfs.WithSupplyTimeThreshold(0.02);
// // currentConfs.WithStatorCurrentLimit(gains_helper->current_limit_.value());
// //     currentConfs.WithStatorCurrentLimitEnable(true);

// //     voltageConfs.PeakForwardVoltage = 12.0;
// //     voltageConfs.PeakReverseVoltage = -12.0;

// //     deviceConfigs.WithVoltage(voltageConfs);
// //     deviceConfigs.WithCurrentLimits(currentConfs);

// //     CheckOk(obj, configurator_.Apply(deviceConfigs));

// //     if (gains_helper != nullptr) {
// //       gains_helper->Write(configurator_, deviceConfigs, gains_cache_,
// false);
// //     }

// //     esc_.OptimizeBusUtilization();

// //     esc_.GetPosition().SetUpdateFrequency(50_Hz);
// //     esc_.GetVelocity().SetUpdateFrequency(50_Hz);
// //     esc_.GetSupplyCurrent().SetUpdateFrequency(10_Hz);
// //     // esc_.GetSupplyVoltage().SetUpdateFrequency(10_Hz);
// //     // esc_.GetForwardLimit().SetUpdateFrequency(50_Hz);
// //     // esc_.GetReverseLimit().SetUpdateFrequency(50_Hz);

// //     return 0;
// //   };

// //   int Reset(units::millisecond_t timeout) {
// //     if (!setup) return -1;

// //     ctre::configs::VoltageConfigs voltageConfs{};
// //     ctre::configs::CurrentLimitsConfigs currentConfs{};

// // currentConfs.WithSupplyCurrentLimit(gains_helper->current_limit_.value()
// //     /
// //                                         2.0);
// //     currentConfs.WithSupplyCurrentThreshold(
// //         gains_helper->current_limit_.value());
// //     currentConfs.WithSupplyTimeThreshold(0.02);
// // currentConfs.WithStatorCurrentLimit(gains_helper->current_limit_.value());
// //     currentConfs.WithStatorCurrentLimitEnable(true);

// //     deviceConfigs.WithVoltage(voltageConfs);
// //     deviceConfigs.WithCurrentLimits(currentConfs);

// //     CheckOk(obj, configurator_.Apply(deviceConfigs));

// //     if (gains_helper != nullptr) {
// //       gains_helper->Write(configurator_, deviceConfigs, gains_cache_,
// false);
// //     }

// //     esc_.OptimizeBusUtilization();

// //     esc_.GetPosition().SetUpdateFrequency(50_Hz);
// //     esc_.GetVelocity().SetUpdateFrequency(50_Hz);
// //     esc_.GetSupplyCurrent().SetUpdateFrequency(10_Hz);
// //     // esc_.GetSupplyVoltage().SetUpdateFrequency(10_Hz);
// //     // esc_.GetForwardLimit().SetUpdateFrequency(50_Hz);
// //     // esc_.GetReverseLimit().SetUpdateFrequency(50_Hz);

// //     return 0;
// //   };

// //   void SetupConverter(X conv) {
// //     if (!setup) return;

// //     conv_.ChangeConversion(conv);
// //   };

// //   void ZeroEncoder(X pos) {
// //     if (!setup) return;

// //     esc_.SetPosition(units::turn_t(conv_.RealToNativePosition(pos)));
// //   };

// //   void ZeroEncoder() {
// //     if (!setup) return;

// //     esc_.SetPosition(0.0_tr);
// //   };

// //   void Write(ControlMode mode, double output) {
// //     if (!setup) return;

// //     if (esc_.HasResetOccurred()) {
// //       Reset(CANTimeout);
// //       esc_.ClearStickyFaults();
// //     }

// //     // CheckOk(obj, configurator_.Apply(deviceConfigs));

// //     if (mode == ControlMode::Percent) {
// //       auto peak_output = gains_helper->peak_output_.value();

// //       double value = std::max(output, -peak_output);
// //       value = std::min(value, +peak_output);

// //       esc_.Set(value);
// //     }
// //   };

// //   void Write(ControlMode mode, V output) {
// //     if (!setup) return;

// //     if (esc_.HasResetOccurred()) {
// //       Reset(CANTimeout);
// //       esc_.ClearStickyFaults();
// //     }

// //     if (gains_helper != nullptr) {
// //       gains_helper->Write(configurator_, deviceConfigs, gains_cache_,
// false);
// //     }

// //     // CheckOk(obj, configurator_.Apply(deviceConfigs));
// //     if (mode == ControlMode::Velocity) {
// //       ctre::controls::VelocityDutyCycle cntrl{
// //           units::turns_per_second_t(conv_.RealToNativeVelocity(output))};
// //       esc_.SetControl(cntrl);
// //     }
// //   };

// //   void Write(ControlMode mode, X output) {
// //     if (!setup) return;

// //     if (esc_.HasResetOccurred()) {
// //       Reset(CANTimeout);
// //       esc_.ClearStickyFaults();
// //     }

// //     if (gains_helper != nullptr) {
// //       gains_helper->Write(configurator_, deviceConfigs, gains_cache_,
// false);
// //     }

// //     // CheckOk(obj, configurator_.Apply(deviceConfigs));

// //     if (mode == ControlMode::Position) {
// //       ctre::controls::PositionDutyCycle cntrl{
// //           units::turn_t(conv_.RealToNativePosition(output))};
// //       esc_.SetControl(cntrl);
// //     }
// //   };

// //   V GetVelocity() {
// //     if (!setup) return units::make_unit<V>(0);

// //     return
// conv_.NativeToRealVelocity(esc_.GetVelocity().GetValueAsDouble());
// //   };

// //   X GetPosition() {
// //     if (!setup) return units::make_unit<X>(0);

// //     return
// conv_.NativeToRealPosition(esc_.GetPosition().GetValueAsDouble());
// //   };

// //   bool VerifyConnected() { return esc_.IsAlive(); };

// //   bool GetInverted() {
// //     if (!setup) return false;

// //     esc_.GetInverted();
// //     return esc_.GetInverted();
// //   };

// //   void SetInverted(bool invert) {
// //     if (!setup) return;

// //     ctre::configs::MotorOutputConfigs motorConfs{};
// //     ctre::configs::Slot0Configs pidConfs{};

// //     motorConfs.WithInverted(invert);

// //     deviceConfigs.WithMotorOutput(motorConfs);

// //     CheckOk(obj, configurator_.Apply(deviceConfigs));
// //   };

// //   units::ampere_t GetCurrent() {
// //     if (!setup) return 0_A;

// //     return 0_A;
// //   };

// //  private:
// //   Loggable obj;
// //   ControlGainsHelper* gains_helper;

// //   ctre::phoenix6::hardware::TalonFX esc_;
// //   ctre::phoenix6::configs::TalonFXConfigurator& configurator_;

// //   ctre::phoenix6::configs::TalonFXConfiguration deviceConfigs{};

// //   ControlGains gains_cache_;

// //   util::Converter<X> conv_{util::kTalonPeriod, util::kTalonFXSensorTicks,
// //                            units::make_unit<X>(1.0)};

// //   bool setup = false;

// //   void CheckOk(Loggable& loggable, ctre::phoenix::StatusCode err,
// //                std::string field = "TalonFX Config") {
// //     if (!(err.IsOK() || err.IsWarning())) {
// //       loggable.Warn("Unable to update {}", field);
// //     }
// //   };
// };
};  // namespace frc846::control

#endif