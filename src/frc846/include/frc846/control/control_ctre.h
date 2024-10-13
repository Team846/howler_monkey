#pragma once

#include "controlbase.h"
#include "units/math.h"

namespace frc846::control {

/*
 * CTRE Talon FX
 */

template <typename X>
class TalonFXController : public BaseESC<X> {
  using V = units::unit_t<
      units::compound_unit<typename X::unit_type,  // the velocity unit is equal
                                                   // to the position unit / 1_s
                           units::inverse<units::second>>>;

  using A = units::ampere_t;  // current

 public:
  TalonFXController(frc846::base::Loggable& parent, int canID,
                    frc846::control::ConfigHelper& config_helper,
                    frc846::control::HardLimitsConfigHelper<X>& hard_limits)
      : parent_{parent},
        canID_{canID},
        config_helper_{config_helper},
        hard_limits_{hard_limits},
        esc_{nullptr},
        configurator_{nullptr} {}

  void OverrideInvert(bool invert) {
    if (!esc_) return;

    esc_->SetInverted(invert);
  }

  bool VerifyConnected() override {
    if (esc_ == nullptr) return false;

    return esc_->IsAlive();
  }

  void WriteDC(double output) override {
    if (esc_ != nullptr) {
      if (!canopt.check(SimpleCANOpt::duty_cycle_type, output)) return;

      ctre::controls::DutyCycleOut cntrl{
          std::max(std::min(output, hard_limits_.get().peak_output_forward),
                   hard_limits_.get().peak_output_reverse)};

      canopt.registerSuccess(CheckOK(esc_->SetControl(cntrl)));
    }
  }

  void WriteVelocity(V output) override {
    if (esc_ != nullptr) {
      if (!canopt.check(SimpleCANOpt::velocity_type,
                        output.template to<double>()))
        return;

      if (config_helper_.hasGainsChanged() && configurator_ != nullptr) {
        ctre::configs::Slot0Configs pidConfs{};

        auto g = config_helper_.getGains();
        pidConfs.WithKP(g.kP);
        pidConfs.WithKD(g.kD);
        pidConfs.WithKS(g.kF);

        deviceConfigs.WithSlot0(pidConfs);

        CheckOK(configurator_->Apply(deviceConfigs));
      }

      ctre::controls::VelocityDutyCycle cntrl{
          1_tr / 1_s * output.template to<double>() /
          config_helper_.getMotorConfig().gear_ratio};

      canopt.registerSuccess(CheckOK(esc_->SetControl(cntrl)));
    }
  }

  void WritePosition(X output) override {
    if (esc_ != nullptr) {
      if (!canopt.check(SimpleCANOpt::velocity_type,
                        output.template to<double>()))
        return;

      if (config_helper_.hasGainsChanged() && configurator_ != nullptr) {
        ctre::configs::Slot0Configs pidConfs{};

        auto g = config_helper_.getGains();
        pidConfs.WithKP(g.kP);
        pidConfs.WithKD(g.kD);
        pidConfs.WithKS(g.kF);

        deviceConfigs.WithSlot0(pidConfs);

        CheckOK(configurator_->Apply(deviceConfigs));
      }

      ctre::controls::PositionDutyCycle cntrl{
          1_tr * output.template to<double>() /
          config_helper_.getMotorConfig().gear_ratio};

      canopt.registerSuccess(CheckOK(esc_->SetControl(cntrl)));
    }
  }

  void ZeroEncoder(X val) {
    if (esc_ == nullptr) return;

    esc_->SetPosition(1_tr * val.template to<double>() /
                      config_helper_.getMotorConfig().gear_ratio);
  }

  V GetVelocity() override {
    if (esc_ == nullptr) return units::make_unit<V>(0.0);

    return units::make_unit<V>(esc_->GetVelocity().GetValueAsDouble() *
                               config_helper_.getMotorConfig().gear_ratio);
  }

  double GetVelocityPercentage() override {
    if (esc_ == nullptr) return 0.0;

    return esc_->GetVelocity().GetValue() /
           DefaultSpecifications::free_speed_kraken;
  }

  X GetPosition() override {
    if (esc_ == nullptr) return units::make_unit<X>(0.0);

    return units::make_unit<X>(esc_->GetPosition().GetValueAsDouble() *
                               config_helper_.getMotorConfig().gear_ratio);
  }

  units::ampere_t GetCurrent() override {
    if (esc_ == nullptr) return 0_A;

    return units::ampere_t(esc_->GetSupplyCurrent().GetValueAsDouble());
  }

  bool GetInverted() override {
    if (esc_ != nullptr) {
      return esc_->GetInverted();
    }

    return false;
  }

  void SetVoltageCompensationAuton(bool auton) override {
    if (configurator_ != nullptr) {
      ctre::VoltageConfigs voltageConfs{};

      auto vcomp =
          auton ? config_helper_.getMotorConfig().auton_voltage_compensation
                : config_helper_.getMotorConfig().voltage_compensation;
      voltageConfs.WithPeakForwardVoltage(vcomp.template to<double>())
          .WithPeakReverseVoltage(-vcomp.template to<double>());

      deviceConfigs.WithVoltage(voltageConfs);

      CheckOK(configurator_->Apply(deviceConfigs));
    }
  }

  void Init() { esc_ = new ctre::phoenix6::hardware::TalonFX(canID_); }

  int Configure(std::vector<DataTag> data_tags) {
    parent_.Log("Attempting to configure TalonFX with CAN ID {}", canID_);

    if (!esc_) {
      parent_.Error("Failed to configure TalonFX with CAN ID {}", canID_);
      return 1;
    }

    if (!esc_->IsAlive()) {
      delete esc_;
      return 1;
    }

    auto motor_config = config_helper_.getMotorConfig();

    SetVoltageCompensationAuton(true);

    esc_->SetInverted(motor_config.invert);

    esc_->SetNeutralMode(
        motor_config.idle_mode == frc846::control::MotorIdleMode::kDefaultCoast
            ? ctre::phoenix6::signals::NeutralModeValue::Coast
            : ctre::phoenix6::signals::NeutralModeValue::Brake);

    ctre::configs::CurrentLimitsConfigs currentConfs;
    currentConfs.WithSupplyCurrentLimitEnable(true);
    currentConfs.WithSupplyCurrentLimit(
        motor_config.current_limiting.target_threshold.template to<double>());
    currentConfs.WithSupplyTimeThreshold(
        motor_config.current_limiting.peak_time_threshold
            .template to<double>());

    deviceConfigs.WithCurrentLimits(currentConfs);

    if (hard_limits_.get().usingPositionLimits) {
      ctre::configs::SoftwareLimitSwitchConfigs limitConfs;
      limitConfs.WithForwardSoftLimitEnable(true);
      limitConfs.WithForwardSoftLimitThreshold(
          hard_limits_.get().forward.template to<double>() /
          motor_config.gear_ratio);
      limitConfs.WithReverseSoftLimitEnable(true);
      limitConfs.WithReverseSoftLimitThreshold(
          hard_limits_.get().reverse.template to<double>() /
          motor_config.gear_ratio);
    }

    ctre::configs::Slot0Configs pidConfs{};

    auto g = config_helper_.getGains();
    pidConfs.WithKP(g.kP);
    pidConfs.WithKD(g.kD);
    pidConfs.WithKS(g.kF);

    deviceConfigs.WithSlot0(pidConfs);

    configurator_ = &esc_->GetConfigurator();
    this->Q(parent_,
            [&]() { return CheckOK(configurator_->Apply(deviceConfigs)); });

    this->Q(parent_, [&]() {
      return CheckOK(esc_->OptimizeBusUtilization(1_ms * DNC::CANTimeout));
    });

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kVelocityData) !=
        data_tags.end()) {
      CheckOK(esc_->GetVelocity().SetUpdateFrequency(50_Hz));
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kPositionData) !=
        data_tags.end()) {
      CheckOK(esc_->GetPosition().SetUpdateFrequency(50_Hz));
    }

    if (std::find(data_tags.begin(), data_tags.end(), DataTag::kCurrentData) !=
        data_tags.end()) {
      CheckOK(esc_->GetSupplyCurrent().SetUpdateFrequency(10_Hz));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    return 0;
  }

 private:
  bool CheckOK(ctre::phoenix::StatusCode err) {
    if (!(err.IsOK() || err.IsWarning())) {
      parent_.Error(
          "TalonFX Motor Controller Error [{}]. CAN ID {}.",
          ErrorParsing::parseErrorCode(ErrorParsing::getErrorCode(err)),
          canID_);
      return false;
    }
    return true;
  }

  frc846::base::Loggable& parent_;
  int canID_;

  frc846::control::ConfigHelper& config_helper_;
  frc846::control::HardLimitsConfigHelper<X>& hard_limits_;
  ctre::phoenix6::hardware::TalonFX* esc_;
  ctre::phoenix6::configs::TalonFXConfigurator* configurator_;

  ctre::phoenix6::configs::TalonFXConfiguration deviceConfigs{};

  SimpleCANOpt canopt{};
};
};  // namespace frc846::control
