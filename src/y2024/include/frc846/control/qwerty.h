#ifndef FRC846_2CONTROL_H_
#define FRC846_2CONTROL_H_

#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <units/current.h>

#include <algorithm>
#include <ctre/phoenix6/TalonFX.hpp>
#include <initializer_list>
#include <variant>

#include "frc846/control/config.h"
#include "frc846/ctre_namespace.h"
#include "frc846/loggable.h"
#include "frc846/util/conversions.h"
FRC846_CTRE_NAMESPACE()

namespace frc846::control {

/*
 * Simple can optimization class using last message cache
 */

class SimpleCANOpt {
 public:
  static constexpr unsigned int duty_cycle_type = 0;
  static constexpr unsigned int velocity_type = 1;
  static constexpr unsigned int position_type = 2;

  SimpleCANOpt() {}

  /* returns true either if the new message is not
   * the same as the cached one or if an error occured in sending the last
   * message
   */
  bool check(int messageType, double messageValue) {
    if (lastFailure) return true;

    if (messageType != lastMessageType ||
        std::abs(messageValue - lastMessageValue) > 0.005) {
      lastMessageType = messageType;
      lastMessageValue = messageValue;
      return true;
    }

    return false;
  }

  bool registerSuccess(bool success) { lastFailure = !success; }

 private:
  unsigned int lastMessageType;
  double lastMessageValue;
  bool lastFailure = false;
};

/*
 * Common class for all custom ESC wrappers
 */

template <typename X>
class ElectronicSpeedController {
  using V = units::unit_t<
      units::compound_unit<typename X::unit_type,  // the velocity unit is equal
                                                   // to the position unit / 1_s
                           units::inverse<units::second>>>;

  using A = units::ampere_t;  // current

 public:
  virtual int Configure() = 0;  // setup motor

  virtual void SetVoltageCompensationAuton(
      bool auton) = 0;  // to switch between voltage compensation presets for
                        // autonomous and teleop

  virtual void ZeroEncoder(X pos) = 0;

  virtual void WriteDC(double output) = 0;

  virtual void WriteVelocity(V output) = 0;

  virtual void WritePosition(X output) = 0;

  virtual V GetVelocity() = 0;

  virtual X GetPosition() = 0;

  virtual bool VerifyConnected() = 0;

  virtual bool GetInverted() = 0;

  virtual units::ampere_t GetCurrent() = 0;

 protected:
  constexpr unsigned int getErrorCode(rev::REVLibError err) {
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

  constexpr unsigned int getErrorCode(ctre::phoenix::StatusCode& err) {
    if (err.IsOK()) {
      return 0;
    } else if (err.IsWarning()) {
      return 11;
    } else if (err.IsError()) {
      return 1;
    }

    return -1;
  }

  constexpr std::string parseErrorCode(unsigned int err) {
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
};

/*
 * Common class for Spark MAX and FLEX
 */

template <typename X>
class BaseSparkController : ElectronicSpeedController<X> {
 public:
  BaseSparkController() = delete;
  BaseSparkController(frc846::Subsystem* parent, int canID,
                      frc846::control::ConfigHelper& config_helper,
                      frc846::control::HardLimitsConfigHelper<X>& hard_limits,
                      std::optional<rev::SparkRelativeEncoder>* encoder,
                      std::optional<rev::SparkPIDController>* pid_controller)
      : parent_{parent},
        canID_{canID},
        config_helper_{config_helper},
        hard_limits_{hard_limits},
        encoder_{encoder},
        pid_controller{pid_controller} {}

  void ZeroEncoder(X value) override {
    if (encoder_->has_value()) {
      CheckOK(encoder_->value().SetPosition(
          value.to<double>() / config_helper_.getMotorConfig().gear_ratio));
    }
  }

  void WriteDC(double output) override {
    if (pid_controller_.has_value()) {
      if (!canopt.check(SimpleCANOpt::duty_cycle_type, output)) return;

      canopt.registerSuccess(CheckOK(pid_controller_->value().SetReference(
          std::max(std::min(output, hard_limits_.get().peak_output_forward),
                   hard_limits_.get().peak_output_reverse),
          rev::ControlType::kDutyCycle)));
    }
  }

  void WriteVelocity(V output) override {
    if (pid_controller_.has_value()) {
      if (config_helper_
              .hasGainsChanged()) {  // this allows for pid to be tuned without
                                     // having to restart robot code
        auto g = config_helper_.getGains();
        CheckOK(pid_controller_->value().SetP(g.kP));
        CheckOK(pid_controller_->value().SetD(g.kD));
        if (config_helper_.getMotorConfig().usingDynamicFF)
          CheckOK(pid_controller_->value().SetFF(g.kF));
      }

      if (!canopt.check(SimpleCANOpt::velocity_type, output.to<double>()))
        return;

      // DO NOT USE REV SMART VELOCITY
      canopt.registerSuccess(CheckOK(pid_controller_->value().SetReference(
          output.to<double>() / config_helper_.getMotorConfig().gear_ratio *
              60.0,                        // 60.0 seconds to a minute
          rev::ControlType::kVelocity)));  // native time period is in minutes
    }
  }

  void WritePosition(X output) override {
    if (pid_controller_.has_value()) {
      if (config_helper_
              .hasGainsChanged()) {  // this allows for pid to be tuned without
                                     // having to restart robot code
        auto g = config_helper_.getGains();
        CheckOK(pid_controller_->value().SetP(g.kP));
        CheckOK(pid_controller_->value().SetD(g.kD));
        if (config_helper_.getMotorConfig().usingDynamicFF)
          CheckOK(pid_controller_->value().SetFF(g.kF));
      }

      if (!canopt.check(SimpleCANOpt::position_type, output.to<double>()))
        return;

      // DO NOT USE REV SMART MOTION
      canopt.registerSucess(CheckOK(pid_controller_->value().SetReference(
          output.to<double>() / config_helper_.getMotorConfig().gear_ratio,
          rev::ControlType::kPosition)));
    }
  }

  V GetVelocity() override {
    if (!encoder_->has_value()) return units::make_unit<V>(0.0);

    return encoder_->value().GetVelocity() *
           config_helper_.getMotorConfig().gear_ratio /
           60.0;  // 60.0 seconds to a minute
                  // native time period is in minutes
  }

  X GetPosition() override {
    if (!encoder_->has_value()) return units::make_unit<X>(0.0);

    return encoder_->value().GetPosition() *
           config_helper_.getMotorConfig().gear_ratio;
  }

 protected:
  bool CheckOK(rev::REVLibError err) {
    if (!err = rev::REVLibError::kOk) {
      parent_.Error("REV Motor Controller Error [{}]. CAN ID {}.",
                    parseErrorCode(getErrorCode(err)), canID_);
      return false;
    }
    return true;
  }

  int ConfigureSettings(rev::CANSparkBase& esc) {
    if (!CheckOK(esc.RestoreFactoryDefaults(true))) {
      return 1;  // does not configure controller in this case
    }

    /* Setting configs */

    esc.SetCANTimeout(20.0);  // If this value is higher, issues on CAN bus can
                              // cause much worse loop overruns

    auto motor_config = config_helper_.getMotorConfig();

    CheckOK(esc.EnableVoltageCompensation(
        motor_config.auton_voltage_compensation_
            .to<double>()));  // voltage compensation sets the maximum voltage
                              // that the controller will output. 10-12V is good
                              // for consistency during autonomous, but the
                              // value can be above 12V for teleop

    CheckOK(esc.SetInverted(motor_config.invert));

    CheckOK(esc.SetIdleMode(motor_config.idle_mode ==
                                    frc846::control::IdleMode::kCoast
                                ? rev::CANSparkBase::IdleMode::kCoast
                                : rev::CANSparkBase::IdleMode::kBrake));

    if (motor_config.withRampRate) {
      CheckOK(esc.SetOpenLoopRampRate(
          motor_config.rampTime
              .to<double>()));  // if our can bus isn't overloaded, we
                                // should use custom motion profiles instead
      CheckOK(esc.SetClosedLoopRampRate(motor_config.rampTime.to<double>()));
    }

    CheckOK(esc.SetSmartCurrentLimit(
        motor_config.current_limiting.target_threshold
            .to<double>()));  // REV current limit uses PID to control
                              // current if it exceeds output. Reactive
                              // system. Happens on board controller at high
                              // rate.

    if (hard_limits_.get().usingPositionLimits) {  // position limiting
      esc.SetSoftLimit(
          rev::CANSparkBase::SoftLimitDirection::kForward,
          hard_limits_.get().forward.to<double>() / motor_config.gear_ratio);
      esc.SetSoftLimit(
          rev::CANSparkBase::SoftLimitDirection::kReverse,
          hard_limits_.get().reverse.to<double>() / motor_config.gear_ratio);
    }
    return 0;
  }

  void ConfigureGains() {
    auto motor_config = config_helper_.getMotorConfig();

    CheckOK(pid_controller_->value().SetP(config_helper_.getGains().kP));
    CheckOK(pid_controller_->value().SetD(config_helper_.getGains().kD));
    if (!motor_config.usingDynamicFF)
      CheckOK(pid_controller_->value().SetFF(config_helper_.getGains().kF));

    // also setting forward and reverse peak outputs for velocity and position
    // controllers. check if this applies to dc control as well.
    CheckOK(pid_controller_->value().SetOutputRange(
        hard_limits_.get().peak_output_reverse,
        hard_limits_.get().peak_output_forward));
  }

  void ConfigureStatusFrames(rev::CANSparkBase& esc,
                             std::vector<DataTag> data_tags) {
    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kLeader) ==
        data_tags.end()) {  // If NOT Leader
      if (std::find(data_tags.begin(), data_tags.end(), DataTag::kFaultData) !=
          data_tags.end()) {  // If looking for fault data (such as sensor
                              // configured as limit switch)
        CheckOK(esc.SetPeriodicFramePeriod(
            rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 20));
      } else {
        CheckOK(esc.SetPeriodicFramePeriod(
            rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 500));
      }
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kVelocityData) ==
            data_tags.end() &&
        std::find(data_tags.begin(), data_tags.end(), DataTag::kCurrentData) ==
            data_tags.end()) {  // If velocity and current data are unnecessary
      CheckOK(esc.SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 65535));
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kPositionData) ==
        data_tags.end()) {  // If position data is unnecessary
      CheckOK(esc.SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 65535));
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kSensorData) ==
        data_tags.end()) {  // If analog sensor data is unnecessary
      CheckOK(esc.SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 65535));
    }

    CheckOK(esc.SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 65535));

    CheckOK(esc.SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 65535));

    CheckOK(esc.SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 65535));
  }

 private:
  frc846::Subsystem* parent_;
  int canID_;

  frc846::control::ConfigHelper& config_helper_;
  frc846::control::HardLimitsConfigHelper<X>& hard_limits_;
  std::optional<rev::SparkMaxRelativeEncoder>*
      encoder_;  // optionals in case the motor is never configured
  std::optional<rev::SparkPIDController>* pid_controller_;

  SimpleCANOpt canopt{};
};

/*
 * REV Spark MAX
 */

template <typename X>
class SparkMAXController : BaseSparkController<X> {
 public:
  SparkMAXController(frc846::Subsystem* parent, int canID,
                     ConfigHelper& config_helper,
                     HardLimitsConfigHelper<X>& hard_limits_configs)
      : parent_{parent},
        canID_{canID},
        config_helper_{config_helper},
        hard_limits_{hard_limits_configs},
        encoder_{std::nullopt},
        pid_controller_{std::nullopt},
        BaseSparkController{parent_, config_helper, hard_limits_configs,
                            encoder_, pid_controller_} {}

  int Configure(std::vector<DataTag> data_tags) override {
    rev::CANSparkMax esc{canID_, rev::CANSparkBase::MotorType::kBrushless};

    /* Main Settings */
    if (ConfigureSettings(esc) == 1) {
      parent_.Error("Was unable to configure Spark MAX id {}", canID_);
      return 1;
    };

    /* Setting gains */
    pid_controller_.emplace(esc.GetPIDController());

    ConfigureGains();

    /* Disable Status Frames */
    ConfigureStatusFrames(data_tags);

    /* etc */
    esc_.emplace(esc);
    encoder_.emplace(
        esc.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42));

    return 0;
  }

  bool VerifyConnected() override {
    if (esc_.has_value()) {
      esc_.value().GetFirmwareVersion();  // Don't remove
      return esc_.value().GetFirmwareVersion() != 0;
    }
  }

  bool GetInverted() override {
    if (esc_.has_value()) {
      return esc_.value().GetInverted();
    }
    return false;
  }

  units::ampere_t GetCurrent() override {
    if (esc_.has_value()) {
      return units::ampere_t(esc_.value().GetOutputCurrent());
    }
    return 0_A;
  }

  void SetVoltageCompensationAuton(bool auton) override {
    if (esc_.has_value()) {
      CheckOK(esc_.value().EnableVoltageCompensation(
          auton ? config_helper_.getMotorConfig()
                      .auton_voltage_compensation.to<double>()
                : config_helper_.getMotorConfig()
                      .voltage_compensation
                      .to<double>()));  // switching between autonomous
                                        // and teleop presets for
                                        // voltage compensation
    }
  }

 private:
  int canID_;
  frc846::Subsystem* parent_;
  frc846::control::ConfigHelper& config_helper_;
  frc846::control::HardLimitsConfigHelper<X>& hard_limits_;
  std::optional<rev::SparkMaxRelativeEncoder> encoder_;
  std::optional<rev::SparkPIDController> pid_controller_;

  std::optional<rev::CANSparkMax> esc_;
};

/*
 * REV Spark FLEX
 */

template <typename X>
class SparkFLEXController : BaseSparkController<X> {
 public:
  SparkFLEXController(frc846::Subsystem* parent, int canID,
                      ConfigHelper& config_helper,
                      HardLimitsConfigHelper<X>& hard_limits_configs)
      : parent_{parent},
        canID_{canID},
        config_helper_{config_helper},
        hard_limits_{hard_limits_configs},
        encoder_{std::nullopt},
        pid_controller_{std::nullopt},
        BaseSparkController{parent_, config_helper, hard_limits_configs,
                            encoder_, pid_controller_} {}

  int Configure(std::vector<DataTag> data_tags) override {
    rev::CANSparkFlex esc{canID_, rev::CANSparkBase::MotorType::kBrushless};

    /* Main Settings */
    if (ConfigureSettings(esc) == 1) {
      parent_.Error("Was unable to configure Spark FLEX id {}", canID_);
      return 1;
    };

    /* Setting gains */
    pid_controller_.emplace(esc.GetPIDController());

    ConfigureGains();

    /* Disable Status Frames */
    ConfigureStatusFrames(data_tags);

    /* etc */
    esc_.emplace(esc);
    encoder_.emplace(
        esc.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42));

    return 0;
  }

  bool VerifyConnected() override {
    if (esc_.has_value()) {
      esc_.value().GetFirmwareVersion();  // Don't remove
      return esc_.value().GetFirmwareVersion() != 0;
    }
  }

  bool GetInverted() override {
    if (esc_.has_value()) {
      return esc_.value().GetInverted();
    }
    return false;
  }

  units::ampere_t GetCurrent() override {
    if (esc_.has_value()) {
      return units::ampere_t(esc_.value().GetOutputCurrent());
    }
    return 0_A;
  }

  void SetVoltageCompensationAuton(bool auton) override {
    if (esc_.has_value()) {
      CheckOK(esc_.value().EnableVoltageCompensation(
          auton ? config_helper_.getMotorConfig()
                      .auton_voltage_compensation.to<double>()
                : config_helper_.getMotorConfig()
                      .voltage_compensation
                      .to<double>()));  // switching between autonomous
                                        // and teleop presets for
                                        // voltage compensation
    }
  }

 private:
  int canID_;
  frc846::Subsystem* parent_;
  frc846::control::ConfigHelper& config_helper_;
  frc846::control::HardLimits& hard_limits_;
  std::optional<rev::SparkMaxRelativeEncoder> encoder_;
  std::optional<rev::SparkPIDController> pid_controller_;

  std::optional<rev::CANSparkMax> esc_;
};

/*
 * CTRE Talon FX
 */

template <typename X>
class TalonFXController : ElectronicSpeedController<X> {
 public:
  TalonFXController() = delete;
  TalonFXController(frc846::Subsystem* parent, int canID,
                    frc846::control::ConfigHelper& config_helper,
                    frc846::control::HardLimitsConfigHelper<X>& hard_limits)
      : parent_{parent},
        canID_{canID},
        config_helper_{config_helper},
        hard_limits_{hard_limits} {}

  void WriteDC(double output) override {
    if (esc_.has_value()) {
      if (!canopt.check(SimpleCANOpt::duty_cycle_type, output)) return;

      ctre::controls::DutyCycleOut cntrl{
          std::max(std::min(output, hard_limits_.get().peak_output_forward),
                   hard_limits_.get().peak_output_reverse)};

      canopt.registerSuccess(CheckOK(esc_.value().SetControl(cntrl)));
    }
  }

  void WriteVelocity(V output) override {
    if (esc_.has_value()) {
      if (!canopt.check(SimpleCANOpt::velocity_type, output.to<double>()))
        return;

      if (config_helper_.hasGainsChanged() &&
          configurator_
              .has_value()) {  // this allows for pid to be tuned without
        // having to restart robot code
        ctre::configs::Slot0Configs pidConfs{};

        auto g = config_helper_.getGains();
        pidConfs.WithKP(g.kP);
        pidConfs.WithKD(g.kD);
        if (config_helper_.getMotorConfig().usingDynamicFF)
          pidConfs.WithKS(g.kF);

        deviceConfigs.WithSlot0(pidConfs);

        CheckOK(configurator_.value().Apply(deviceConfigs));
      }

      ctre::controls::VelocityDutyCycle cntrl{
          output / config_helper_.getMotorConfig()
                       .gear_ratio};  // native time period is in seconds to
                                      // minute/second conversion is not
                                      // required (unlike for spark max or flex)

      canopt.registerSuccess(CheckOK(esc_.value().SetControl(cntrl)));
    }
  }

  void WritePosition(X output) override {
    if (esc_.has_value()) {
      if (!canopt.check(SimpleCANOpt::velocity_type, output.to<double>()))
        return;

      if (config_helper_.hasGainsChanged() &&
          configurator_
              .has_value()) {  // this allows for pid to be tuned without
        // having to restart robot code
        ctre::configs::Slot0Configs pidConfs{};

        auto g = config_helper_.getGains();
        pidConfs.WithKP(g.kP);
        pidConfs.WithKD(g.kD);
        if (config_helper_.getMotorConfig().usingDynamicFF)
          pidConfs.WithKS(g.kF);

        deviceConfigs.WithSlot0(pidConfs);

        CheckOK(configurator_.value().Apply(deviceConfigs));
      }

      ctre::controls::PositionDutyCycle cntrl{
          output / config_helper_.getMotorConfig().gear_ratio};

      canopt.registerSuccess(CheckOK(esc_.value().SetControl(cntrl)));
    }
  }

  V GetVelocity() override {
    if (!esc_.has_value()) return units::make_unit<V>(0.0);

    return units::make_unit<V>(
        esc_.value().GetVelocity().GetLatencyCompensatedValue().to<double>() *
        config_helper_.getMotorConfig().gear_ratio);
  }

  X GetPosition() override {
    if (!esc_.has_value()) return units::make_unit<X>(0.0);

    return units::make_unit<X>(
        esc_.value().GetPosition().GetLatencyCompensatedValue().to<double>() *
        config_helper_.getMotorConfig().gear_ratio);
  }

  units::ampere_t GetCurrent() override {
    if (!esc_.has_value()) return 0_A;

    return esc_.value().GetSupplyCurrent().GetLatencyCompensatedValue();
  }

  bool GetInverted() override {
    if (esc_.has_value()) {
      return esc_.value().GetInverted();
    }

    return false;
  }

  void SetVoltageCompensationAuton(bool auton) override {
    if (configurator_.has_value()) {
      ctre::VoltageConfigs voltageConfs{};

      auto vcomp =
          auton ? config_helper_.getMotorConfig().auton_voltage_compensation
                : config_helper_.getMotorConfig().voltage_compensation;
      voltageConfs.WithPeakForwardVoltage(vcomp.to<double>())
          .WithPeakReverseVoltage(-vcomp.to<double>());

      deviceConfigs.WithVoltage(voltageConfs);

      CheckOK(configurator_.value().Apply(deviceConfigs));  // switching
                                                            // between
                                                            // autonomous
                                                            // and teleop
                                                            // presets for
                                                            // voltage
                                                            // compensation
    }
  }

  int Configure() {
    ctre::phoenix6::hardware::TalonFX esc{canID_};
    if (!esc.IsAlive()) {
      return 1;  // does not configure controller in this case
    }

    auto motor_config = config_helper_.getMotorConfig();

    /*
     * General settings
     */

    SetVoltageCompensationAuton(true);

    esc.SetInverted(motor_config.invert);

    esc.SetNeutralMode(motor_config.idle_mode ==
                               frc846::control::IdleMode::kCoast
                           ? ctre::phoenix6::signals::NeutralModeValue::Coast
                           : ctre::phoenix6::signals::NeutralModeValue::Brake);

    ctre::configs::CurrentLimitsConfigs currentConfs;
    currentConfs.WithSupplyCurrentLimitEnable(true);
    currentConfs.WithStatorCurrentLimit(  // Not using stator current limits
        motor_config.current_limiting.target_threshold.to<double>());
    currentConfs.WithSupplyTimeThreshold(
        motor_config.current_limiting.peak_time_threshold.to<double>());

    deviceConfigs.WithCurrentLimits(currentConfs);

    /*
     * Hard limits
     */

    if (hard_limits_.get().usingPositionLimits) {
      ctre::configs::SoftwareLimitSwitchConfigs limitConfs;
      limitConfs.WithForwardSoftLimitEnable(true);
      limitConfs.WithForwardSoftLimitThreshold(hard_limits_.get().forward /
                                               motor_config.gear_ratio);
      limitConfs.WithReverseSoftLimitEnable(true);
      limitConfs.WithReverseSoftLimitThreshold(hard_limits_.get().reverse /
                                               motor_config.gear_ratio);
    }

    ctre::configs::Slot0Configs pidConfs{};

    auto g = config_helper_.getGains();
    pidConfs.WithKP(g.kP);
    pidConfs.WithKD(g.kD);
    if (config_helper_.getMotorConfig().usingDynamicFF) pidConfs.WithKS(g.kF);

    deviceConfigs.WithSlot0(pidConfs);

    configurator_.emplace(esc.GetConfigurator());
    CheckOK(configurator_.value().Apply(deviceConfigs));

    /*
     * Status frames
     */
    CheckOK(esc.OptimizeBusUtilization(20_ms));

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kVelocityData) !=
        data_tags.end()) {
      CheckOK(esc_.GetVelocity().SetUpdateFrequency(50_Hz));
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kPositionData) !=
        data_tags.end()) {
      CheckOK(esc_.GetPosition().SetUpdateFrequency(50_Hz));
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kCurrentData) !=
        data_tags.end()) {
      CheckOK(esc_.GetSupplyCurrent().SetUpdateFrequency(10_Hz));
    }

    esc_.emplace(esc);

    return 0;
  }

 private:
  bool CheckOK(ctre::phoenix::StatusCode err) {
    if
      if (!(err.IsOK() || err.IsWarning())) {
        parent_.Error("TalonFX Motor Controller Error [{}]. CAN ID {}.",
                      parseErrorCode(getErrorCode(err)), canID_);
        return false;
      }
    return true;
  }
  frc846::Subsystem* parent_;
  int canID_;

  frc846::control::ConfigHelper& config_helper_;
  frc846::control::HardLimitsConfigHelper<X>& hard_limits_;
  std::optional<ctre::phoenix6::hardware::TalonFX> esc_;
  std::optional<ctre::phoenix6::configs::TalonFXConfigurator> configurator_;

  ctre::phoenix6::configs::TalonFXConfiguration deviceConfigs{};

  SimpleCANOpt canopt{};
};

}  // namespace frc846::control

#endif