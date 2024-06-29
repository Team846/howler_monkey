#ifndef FRC846_REV_CONTROL_H_
#define FRC846_REV_CONTROL_H_

#include "controlbase.h"

namespace frc846::control {

enum REVSparkType { kSparkMAX, kSparkMAX550, kSparkFLEX };

/*
 * Generic class for both Spark MAX and Spark FLEX
 */

template <typename X>
class REVSparkController : public BaseESC<X> {
  using V = units::unit_t<
      units::compound_unit<typename X::unit_type,  // the velocity unit is equal
                                                   // to the position unit / 1_s
                           units::inverse<units::second>>>;

  using A = units::ampere_t;  // current

 public:
  REVSparkController(frc846::Loggable& parent, int canID,
                     frc846::control::ConfigHelper& config_helper,
                     frc846::control::HardLimitsConfigHelper<X>& hard_limits)
      : parent_{parent},
        canID_{canID},
        config_helper_{config_helper},
        hard_limits_{hard_limits},
        esc_{nullptr},
        encoder_{nullptr},
        pid_controller_{nullptr} {}

  void OverrideInvert(bool invert) override {
    if (!esc_) return;
    esc_->SetInverted(invert);
  }

  bool VerifyConnected() override {
    if (!esc_) return false;

    esc_->GetFirmwareVersion();  // Don't remove
    return esc_->GetFirmwareVersion() != 0;
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
      RefreshPID();

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
      RefreshPID();

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

  void ZeroEncoder(X val) override {
    if (encoder_) {
      CheckOK(
          encoder_->SetPosition(val.template to<double>() /
                                config_helper_.getMotorConfig().gear_ratio));
    }
  }

  V GetVelocity() override {
    if (encoder_ == nullptr) return units::make_unit<V>(0.0);
    return units::make_unit<V>(encoder_->GetVelocity()) *
           config_helper_.getMotorConfig().gear_ratio /
           60.0;  // 60.0 seconds to a minute
                  // native time period is in minutes
  }

  double GetVelocityPercentage() override {
    if (encoder_ == nullptr) return 0.0;

    units::revolutions_per_minute_t max_velocity =
        controller_type_ == kSparkFLEX
            ? DefaultSpecifications::free_speed_vortex
            : DefaultSpecifications::free_speed_neo;

    max_velocity = controller_type_ == kSparkMAX550
                       ? DefaultSpecifications::free_speed_550
                       : max_velocity;

    return encoder_->GetVelocity() * 1_rpm / max_velocity;
  }

  X GetPosition() override {
    if (encoder_ == nullptr) return units::make_unit<X>(0.0);

    return units::make_unit<X>(encoder_->GetPosition()) *
           config_helper_.getMotorConfig().gear_ratio;
  }

  units::ampere_t GetCurrent() override {
    if (esc_) {
      return units::ampere_t(esc_->GetOutputCurrent());
    }
    return 0_A;
  }

  bool GetInverted() override {
    if (esc_) {
      return esc_->GetInverted();
    }
    return false;
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

  int Configure(REVSparkType controller_type, std::vector<DataTag> data_tags) {
    controller_type_ = controller_type;

    if (!CheckOK(esc_->RestoreFactoryDefaults(true))) {
      return 1;  // does not configure controller in this case
    }

    /* Setting configs */

    CheckOK(
        esc_->SetCANTimeout(30.0));  // If this  is higher, issues on CAN
                                     // bus can cause much worse loop overruns

    auto motor_config = config_helper_.getMotorConfig();

    CheckOK(esc_->EnableVoltageCompensation(
        motor_config.auton_voltage_compensation
            .to<double>()));  // voltage compensation sets the maximum voltage
                              // that the controller will output. 10-12V is good
                              // for consistency during autonomous, but the
                              //  can be above 12V for teleop

    esc_->SetInverted(motor_config.invert);

    CheckOK(esc_->SetIdleMode(
        motor_config.idle_mode == frc846::control::MotorIdleMode::kDefaultCoast
            ? rev::CANSparkBase::IdleMode::kCoast
            : rev::CANSparkBase::IdleMode::kBrake));

    if (motor_config.withRampRate) {
      CheckOK(esc_->SetOpenLoopRampRate(
          motor_config.rampTime
              .to<double>()));  // if our can bus isn't overloaded, we
                                // should use custom motion profiles instead

      CheckOK(esc_->SetClosedLoopRampRate(motor_config.rampTime.to<double>()));
    }

    CheckOK(esc_->SetSmartCurrentLimit(
        motor_config.current_limiting.target_threshold
            .to<double>()));  // REV current limit uses PID to control
                              // current if it exceeds output. Reactive
                              // system. Happens on board controller at high
                              // rate.

    if (hard_limits_.get().usingPositionLimits) {  // position limiting
      esc_->SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward,
                         hard_limits_.get().forward.template to<double>() /
                             motor_config.gear_ratio);
      esc_->SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse,
                         hard_limits_.get().reverse.template to<double>() /
                             motor_config.gear_ratio);
    }

    RefreshPID(true);

    DisableStatusFrames(data_tags);

    return 0;
  }

  rev::CANSparkBase* getESC() { return esc_; }

 private:
  void RefreshPID(bool ignore_cache = false) {
    if (ignore_cache ||
        config_helper_
            .hasGainsChanged()) {  // this allows for pid to be tuned without
                                   // having to restart robot code
      auto g = config_helper_.getGains();
      CheckOK(pid_controller_->SetP(g.kP));
      CheckOK(pid_controller_->SetD(g.kD));
      CheckOK(pid_controller_->SetFF(g.kF));
    }
  }

  void DisableStatusFrames(std::vector<DataTag> data_tags) {
    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kLeader) ==
        data_tags.end()) {  // If NOT Leader
      if (std::find(data_tags.begin(), data_tags.end(), DataTag::kFaultData) !=
          data_tags.end()) {  // If looking for fault data (such as sensor
                              // configured as limit switch)
        CheckOK(esc_->SetPeriodicFramePeriod(
            rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 20));
      } else {
        CheckOK(esc_->SetPeriodicFramePeriod(
            rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 500));
      }
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kVelocityData) ==
            data_tags.end() &&
        std::find(data_tags.begin(), data_tags.end(), DataTag::kCurrentData) ==
            data_tags.end()) {  // If velocity and current data are unnecessary
      CheckOK(esc_->SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 65535));
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kPositionData) ==
        data_tags.end()) {  // If position data is unnecessary
      CheckOK(esc_->SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 65535));
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kSensorData) ==
        data_tags.end()) {  // If analog sensor data is unnecessary
      CheckOK(esc_->SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 65535));
    }

    CheckOK(esc_->SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 65535));

    CheckOK(esc_->SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 65535));

    CheckOK(esc_->SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 65535));
  }

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

  frc846::Loggable& parent_;
  int canID_;

  frc846::control::ConfigHelper& config_helper_;
  frc846::control::HardLimitsConfigHelper<X>& hard_limits_;

  rev::CANSparkBase* esc_;
  rev::SparkRelativeEncoder* encoder_;
  rev::SparkPIDController* pid_controller_;

  frc846::control::REVSparkType controller_type_ = kSparkMAX;

  SimpleCANOpt canopt{};
};
};  // namespace frc846::control

#endif
