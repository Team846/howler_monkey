#ifndef FRC846_2CONTROL_H_
#define FRC846_2CONTROL_H_

#include "qwertybase.h"

namespace frc846::control {

/*
 * Common class for Spark MAX and FLEX
 */

template <typename X>
class BaseSparkController : public BaseESC<X> {
  using V = units::unit_t<
      units::compound_unit<typename X::unit_type,  // the velocity unit is equal
                                                   // to the position unit / 1_s
                           units::inverse<units::second>>>;

  using A = units::ampere_t;  // current

 public:
  bool CheckOK(rev::REVLibError err) {
    if (err != rev::REVLibError::kOk) {
      parent_.Error(
          "REV Motor Controller Error [{}]. CAN ID {}.",
          ErrorParsing::parseErrorCode(ErrorParsing::getErrorCode(err)),
          canID_);
      return false;
    }
    return true;
  }

  int ConfigureSettings(rev::CANSparkBase* esc) {
    if (!CheckOK(esc->RestoreFactoryDefaults(true))) {
      return 1;  // does not configure controller in this case
    }

    /* Setting configs */

    esc->SetCANTimeout(20.0);  // If this value is higher, issues on CAN bus can
                               // cause much worse loop overruns

    auto motor_config = config_helper_.getMotorConfig();

    CheckOK(esc->EnableVoltageCompensation(
        motor_config.auton_voltage_compensation
            .to<double>()));  // voltage compensation sets the maximum voltage
                              // that the controller will output. 10-12V is good
                              // for consistency during autonomous, but the
                              // value can be above 12V for teleop

    CheckOK(esc->SetInverted(motor_config.invert));

    CheckOK(esc->SetIdleMode(
        motor_config.idle_mode == frc846::control::MotorIdleMode::kDefaultCoast
            ? rev::CANSparkBase::IdleMode::kCoast
            : rev::CANSparkBase::IdleMode::kBrake));

    if (motor_config.withRampRate) {
      CheckOK(esc->SetOpenLoopRampRate(
          motor_config.rampTime
              .to<double>()));  // if our can bus isn't overloaded, we
                                // should use custom motion profiles instead
      CheckOK(esc->SetClosedLoopRampRate(motor_config.rampTime.to<double>()));
    }

    CheckOK(esc->SetSmartCurrentLimit(
        motor_config.current_limiting.target_threshold
            .to<double>()));  // REV current limit uses PID to control
                              // current if it exceeds output. Reactive
                              // system. Happens on board controller at high
                              // rate.

    if (hard_limits_.get().usingPositionLimits) {  // position limiting
      esc->SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward,
                        hard_limits_.get().forward.template to<double>() /
                            motor_config.gear_ratio);
      esc->SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse,
                        hard_limits_.get().reverse.template to<double>() /
                            motor_config.gear_ratio);
    }
    return 0;
  }

  void ConfigureGains() {
    auto motor_config = config_helper_.getMotorConfig();

    CheckOK(pid_controller_->SetP(config_helper_.getGains().kP));
    CheckOK(pid_controller_->SetD(config_helper_.getGains().kD));
    if (!motor_config.usingDynamicFF)
      CheckOK(pid_controller_->SetFF(config_helper_.getGains().kF));

    // also setting forward and reverse peak outputs for velocity and position
    // controllers. check if this applies to dc control as well.
    CheckOK(pid_controller_->SetOutputRange(
        hard_limits_.get().peak_output_reverse,
        hard_limits_.get().peak_output_forward));
  }

  void ConfigureStatusFrames(rev::CANSparkBase* esc,
                             std::vector<DataTag> data_tags) {
    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kLeader) ==
        data_tags.end()) {  // If NOT Leader
      if (std::find(data_tags.begin(), data_tags.end(), DataTag::kFaultData) !=
          data_tags.end()) {  // If looking for fault data (such as sensor
                              // configured as limit switch)
        CheckOK(esc->SetPeriodicFramePeriod(
            rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 20));
      } else {
        CheckOK(esc->SetPeriodicFramePeriod(
            rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 500));
      }
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kVelocityData) ==
            data_tags.end() &&
        std::find(data_tags.begin(), data_tags.end(), DataTag::kCurrentData) ==
            data_tags.end()) {  // If velocity and current data are unnecessary
      CheckOK(esc->SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 65535));
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kPositionData) ==
        data_tags.end()) {  // If position data is unnecessary
      CheckOK(esc->SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 65535));
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kSensorData) ==
        data_tags.end()) {  // If analog sensor data is unnecessary
      CheckOK(esc->SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 65535));
    }

    CheckOK(esc->SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 65535));

    CheckOK(esc->SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 65535));

    CheckOK(esc->SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 65535));
  }

  BaseSparkController(frc846::Loggable& parent, int canID,
                      frc846::control::ConfigHelper& config_helper,
                      frc846::control::HardLimitsConfigHelper<X>& hard_limits,
                      std::optional<rev::SparkRelativeEncoder*>* encoder,
                      std::optional<rev::SparkPIDController*>* pid_controller)
      : parent_{parent},
        canID_{canID},
        config_helper_{config_helper},
        hard_limits_{hard_limits},
        encoder_{encoder},
        pid_controller_{pid_controller} {}

  void ZeroEncoder(X value) override {
    if (encoder_) {
      CheckOK(
          encoder_->SetPosition(value.template to<double>() /
                                config_helper_.getMotorConfig().gear_ratio));
    }
  }

  void WriteDC(double output) override {
    if (pid_controller_) {
      if (!canopt.check(SimpleCANOpt::duty_cycle_type, output)) return;

      canopt.registerSuccess(CheckOK(pid_controller_->SetReference(
          std::max(std::min(output, hard_limits_.get().peak_output_forward),
                   hard_limits_.get().peak_output_reverse),
          rev::CANSparkBase::ControlType::kDutyCycle)));
    }
  }

  void WriteVelocity(V output) override {
    if (pid_controller_) {
      if (config_helper_
              .hasGainsChanged()) {  // this allows for pid to be tuned without
                                     // having to restart robot code
        auto g = config_helper_.getGains();
        CheckOK(pid_controller_->SetP(g.kP));
        CheckOK(pid_controller_->SetD(g.kD));
        if (!config_helper_.getMotorConfig().usingDynamicFF)
          CheckOK(pid_controller_->SetFF(g.kF));
      }

      if (!canopt.check(SimpleCANOpt::velocity_type,
                        output.template to<double>()))
        return;

      // DO NOT USE REV SMART VELOCITY
      canopt.registerSuccess(CheckOK(pid_controller_->SetReference(
          output.template to<double>() /
              config_helper_.getMotorConfig().gear_ratio *
              60.0,  // 60.0 seconds to a minute
          rev::CANSparkBase::ControlType::kVelocity)));  // native time period
                                                         // is in minutes
    }
  }

  void WritePosition(X output) override {
    if (pid_controller_) {
      if (config_helper_
              .hasGainsChanged()) {  // this allows for pid to be tuned without
                                     // having to restart robot code
        auto g = config_helper_.getGains();
        CheckOK(pid_controller_->SetP(g.kP));
        CheckOK(pid_controller_->SetD(g.kD));
        if (!config_helper_.getMotorConfig().usingDynamicFF)
          CheckOK(pid_controller_->SetFF(g.kF));
      }

      if (!canopt.check(SimpleCANOpt::position_type,
                        output.template to<double>()))
        return;

      // DO NOT USE REV SMART MOTION
      canopt.registerSuccess(CheckOK(pid_controller_->SetReference(
          output.template to<double>() /
              config_helper_.getMotorConfig().gear_ratio,
          rev::CANSparkBase::ControlType::kPosition)));
    }
  }

  V GetVelocity() override {
    if (encoder_ == nullptr) return units::make_unit<V>(0.0);

    return encoder_->GetVelocity() *
           config_helper_.getMotorConfig().gear_ratio /
           60.0;  // 60.0 seconds to a minute
                  // native time period is in minutes
  }

  X GetPosition() override {
    if (encoder_ == nullptr) return units::make_unit<X>(0.0);

    return encoder_->GetPosition() * config_helper_.getMotorConfig().gear_ratio;
  }

 protected:
  void SetOptionals(rev::SparkPIDController* pid_controller,
                    rev::SparkRelativeEncoder* encoder) {
    pid_controller_ = pid_controller;
    encoder_ = encoder;
  }

 private:
  frc846::Loggable& parent_;
  int canID_;

  frc846::control::ConfigHelper& config_helper_;
  frc846::control::HardLimitsConfigHelper<X>& hard_limits_;
  rev::SparkRelativeEncoder* encoder_;
  rev::SparkPIDController* pid_controller_;

  SimpleCANOpt canopt{};
};

/*
 * REV Spark MAX
 */

template <typename X>
class SparkMAXControllerQWERTY : BaseSparkController<X> {
  using V = units::unit_t<
      units::compound_unit<typename X::unit_type,  // the velocity unit is equal
                                                   // to the position unit / 1_s
                           units::inverse<units::second>>>;

  using A = units::ampere_t;  // current

 public:
  SparkMAXControllerQWERTY(frc846::Loggable& parent, int canID,
                           ConfigHelper& config_helper,
                           HardLimitsConfigHelper<X>& hard_limits_configs)
      : parent_{parent},
        canID_{canID},
        config_helper_{config_helper},
        hard_limits_{hard_limits_configs},
        encoder_{std::nullopt},
        pid_controller_{std::nullopt},
        BaseSparkController<X>{parent_, config_helper, hard_limits_configs,
                               encoder_, pid_controller_} {}

  int Configure(std::vector<DataTag> data_tags) override {
    esc_ =
        new rev::CANSparkMax{canID_, rev::CANSparkBase::MotorType::kBrushless};

    /* Main Settings */
    if (this->ConfigureSettings(esc_) == 1) {
      parent_.Error("Was unable to configure Spark FLEX id {}", canID_);
      return 1;
    };

    auto encoder =
        esc_->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
    encoder_ = &encoder;

    auto pid_controller = esc_->GetPIDController();
    pid_controller_ = &pid_controller;

    this->SetOptionals(pid_controller_, encoder_);

    /* Setting gains */

    this->ConfigureGains();

    /* Disable Status Frames */
    this->ConfigureStatusFrames(data_tags);

    return 0;
  }

  bool VerifyConnected() override {
    if (!esc_) return false;

    esc_->GetFirmwareVersion();  // Don't remove
    return esc_->GetFirmwareVersion() != 0;
  }

  bool GetInverted() override {
    if (esc_) {
      return esc_->GetInverted();
    }
    return false;
  }

  units::ampere_t GetCurrent() override {
    if (esc_) {
      return units::ampere_t(esc_->GetOutputCurrent());
    }
    return 0_A;
  }

  void SetVoltageCompensationAuton(bool auton) override {
    if (esc_) {
      this->CheckOK(esc_->EnableVoltageCompensation(
          auton ? config_helper_.getMotorConfig()
                      .auton_voltage_compensation.to<double>()
                : config_helper_.getMotorConfig()
                      .voltage_compensation
                      .to<double>()));  // switching between
                                        // autonomous and teleop
                                        // presets for voltage
                                        // compensation
    }
  }

 private:
  int canID_;
  frc846::Loggable& parent_;
  frc846::control::ConfigHelper& config_helper_;
  frc846::control::HardLimits<X>& hard_limits_;
  rev::SparkRelativeEncoder* encoder_;
  rev::SparkPIDController* pid_controller_;

  rev::CANSparkMax* esc_;
};

/*
 * REV Spark FLEX
 */

template <typename X>
class SparkFLEXControllerQWERTY : BaseSparkController<X> {
  using V = units::unit_t<
      units::compound_unit<typename X::unit_type,  // the velocity unit is equal
                                                   // to the position unit / 1_s
                           units::inverse<units::second>>>;

  using A = units::ampere_t;  // current

 public:
  SparkFLEXControllerQWERTY(frc846::Loggable& parent, int canID,
                            ConfigHelper& config_helper,
                            HardLimitsConfigHelper<X>& hard_limits_configs)
      : parent_{parent},
        canID_{canID},
        config_helper_{config_helper},
        hard_limits_{hard_limits_configs},
        encoder_{std::nullopt},
        pid_controller_{std::nullopt},
        BaseSparkController<X>{parent_, config_helper, hard_limits_configs,
                               encoder_, pid_controller_} {}

  int Configure(std::vector<DataTag> data_tags) override {
    esc_ =
        new rev::CANSparkFlex{canID_, rev::CANSparkBase::MotorType::kBrushless};

    /* Main Settings */
    if (this->ConfigureSettings(esc_) == 1) {
      parent_.Error("Was unable to configure Spark FLEX id {}", canID_);
      return 1;
    };

    auto encoder =
        esc_->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
    encoder_ = &encoder;

    auto pid_controller = esc_->GetPIDController();
    pid_controller_ = &pid_controller;

    this->SetOptionals(pid_controller_, encoder_);

    /* Setting gains */

    this->ConfigureGains();

    /* Disable Status Frames */
    this->ConfigureStatusFrames(data_tags);

    return 0;
  }

  bool VerifyConnected() override {
    if (!esc_) return false;

    esc_->GetFirmwareVersion();  // Don't remove
    return esc_->GetFirmwareVersion() != 0;
  }

  bool GetInverted() override {
    if (esc_) {
      return esc_->GetInverted();
    }
    return false;
  }

  units::ampere_t GetCurrent() override {
    if (esc_) {
      return units::ampere_t(esc_->GetOutputCurrent());
    }
    return 0_A;
  }

  void SetVoltageCompensationAuton(bool auton) override {
    if (esc_) {
      this->CheckOK(esc_->EnableVoltageCompensation(
          auton ? config_helper_.getMotorConfig()
                      .auton_voltage_compensation.to<double>()
                : config_helper_.getMotorConfig()
                      .voltage_compensation
                      .to<double>()));  // switching between
                                        // autonomous and teleop
                                        // presets for voltage
                                        // compensation
    }
  }

 private:
  int canID_;
  frc846::Loggable& parent_;
  frc846::control::ConfigHelper& config_helper_;
  frc846::control::HardLimits<X>& hard_limits_;
  rev::SparkRelativeEncoder* encoder_;
  rev::SparkPIDController* pid_controller_;

  rev::CANSparkFlex* esc_;
};

};  // namespace frc846::control
#endif