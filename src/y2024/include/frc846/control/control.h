#ifndef FRC846_ESC_CONTROL_H_
#define FRC846_ESC_CONTROL_H_

#include <asm-generic/errno.h>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <initializer_list>
#include <variant>

#include "frc846/ctre_namespace.h"
#include "frc846/loggable.h"
#include <units/current.h>
#include "frc846/util/conversions.h"
#include "frc846/control/controlgains.h"

FRC846_CTRE_NAMESPACE()


namespace frc846::control {

static constexpr units::millisecond_t CANTimeout = 50_ms;


enum ControlMode { Percent, Velocity, Position, Current };
enum IdleMode { kCoast, kBrake };

constexpr rev::CANSparkMax::ControlType LocalRevControlMode(ControlMode mode) {
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


template <typename X>
class ElectronicSpeedController {
    using V = units::unit_t<units::compound_unit<typename X::unit_type,
                                                units::inverse<units::second>>>;

    using A = units::ampere_t;

    public:
    virtual int Setup(units::millisecond_t timeout, ControlGainsHelper* gainsHelper,
        bool isInverted = false, IdleMode kIdleMode = IdleMode::kCoast) = 0;

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
    SparkRevController(Loggable parent_, std::string name, int kCANid, rev::CANSparkLowLevel::MotorType type = rev::CANSparkLowLevel::MotorType::kBrushless) :
          obj(parent_, name),
            esc_{kCANid, type}, 
              encoder_{esc_.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)},
              pid_controller_{esc_.GetPIDController()} {
              };

    ~SparkRevController() {
      if (gains_helper != nullptr) {
            delete gains_helper;
      }
    };

    ElectronicSpeedController<X>* that() {
      return this;
    }

    int Setup(units::millisecond_t timeout, ControlGainsHelper* gainsHelper,
        bool isInverted = false, IdleMode kIdleMode = IdleMode::kBrake) {

        if (VerifyConnected()) {
          setup = true;
        } else {
          obj.Error("? controller could NOT be setup");
          return -1;
        }

        if (kIdleMode == kCoast) CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast), "Coast Mode");
        else if (kIdleMode == kBrake) CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake), "Brake Mode");
        
        CheckOk(obj, esc_.SetCANTimeout(timeout.to<double>()), "CAN Timeout");
        esc_.SetInverted(isInverted);
        gains_helper = gainsHelper;

        if (gains_helper != nullptr) {
            gains_helper->Write(pid_controller_, gains_cache_, true);
            pid_controller_.SetOutputRange(-gains_helper->peak_output_.value(), gains_helper->peak_output_.value());
        }

        CheckOk(obj, esc_.SetSmartCurrentLimit(gainsHelper->current_limit_.value()), "Current Limit");
        CheckOk(obj, esc_.EnableVoltageCompensation(12.0), "Voltage Compensation");

        CheckOk(obj, esc_.BurnFlash(), "Burn Flash");
        
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

        if (kIdleMode == kCoast) CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast), "Coast Mode");
        else if (kIdleMode == kBrake) CheckOk(obj, esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake), "Brake Mode");
        CheckOk(obj, esc_.SetCANTimeout(CANTimeout.to<double>()), "CAN Timeout");
        esc_.SetInverted(isInverted);
        gains_helper = gainsHelper;

        if (gains_helper != nullptr) {
             gains_helper->Write(pid_controller_, gains_cache_, true);
             pid_controller_.SetOutputRange(-gains_helper->peak_output_.value(), gains_helper->peak_output_.value());
        }

        CheckOk(obj, esc_.SetSmartCurrentLimit(gainsHelper->current_limit_.value()), "Current Limit");
        CheckOk(obj, esc_.EnableVoltageCompensation(12.0), "Voltage Compensation");

        CheckOk(obj, esc_.BurnFlash(), "Burn Flash");

        return 0;
    };

    
    int Reset(units::millisecond_t timeout) {
      if (!setup) return -1;

      CheckOk(obj, esc_.SetCANTimeout(timeout.to<double>()), "CAN Timeout");

      if (gains_helper != nullptr) {
          gains_helper->Write(pid_controller_, gains_cache_, true);
          pid_controller_.SetOutputRange(-gains_helper->peak_output_.value(), gains_helper->peak_output_.value());
          CheckOk(obj, esc_.SetSmartCurrentLimit(gains_helper->current_limit_.value()), "Current Limit");
      }

      CheckOk(obj, esc_.EnableVoltageCompensation(12.0), "Voltage Compensation");

      CheckOk(obj, esc_.BurnFlash(), "Burn Flash");
      
      return 0;
    };

    void DisableStatusFrames(std::initializer_list<rev::CANSparkLowLevel::PeriodicFrame> frames) {
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

      CheckOk(obj, encoder_.SetPosition(conv_.RealToNativePosition(pos)), "Zero Encoder");
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

        double value = std::max(output, -peak_output);
        value = std::min(value, +peak_output);

        CheckOk(obj, pid_controller_.SetReference(value, LocalRevControlMode(mode)), "Write Duty Cycle");
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
          CheckOk(obj, pid_controller_.SetReference(conv_.RealToNativeVelocity(output), LocalRevControlMode(mode)), "Write Velocity");
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
          CheckOk(obj, pid_controller_.SetReference(conv_.RealToNativePosition(output), LocalRevControlMode(mode)), "Write Position");
      }
    };

    void Write(ControlMode mode, A output) {
      if (!setup) return;
      if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
          Reset(CANTimeout);
          CheckOk(obj, esc_.ClearFaults(), "Clear Faults Post Reset");
      }

      if (mode == ControlMode::Current) {
          CheckOk(obj, pid_controller_.SetReference(output.to<double>(), LocalRevControlMode(mode)), "Write Current");
      }
    };

    void WriteByCurrent(units::ampere_t current) {
      if (!setup) return;

      CheckOk(obj, pid_controller_.SetReference(current.to<double>(), LocalRevControlMode(ControlMode::Current)), "Write Current");
    }

    V GetVelocity() {
      if (!setup) throw std::exception();

      return conv_.NativeToRealVelocity(encoder_.GetVelocity());
    };

    X GetPosition() {
      if (!setup) throw std::exception();

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

    void ConfigurePositionLimits(X forward_limit, X reverse_limit, bool enable = true) {
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

    util::Converter<X> conv_{util::kSparkMAXPeriod, util::kSparkMAXSensorTicks, units::make_unit<X>(1.0)};

    bool setup = false;

    bool usingPositionLimits = false;

    X reverse_position_limit = units::make_unit<X>(0.0);
    X forward_position_limit = units::make_unit<X>(0.0);

    void CheckOk(Loggable& loggable, rev::REVLibError err, std::string field = "?") {
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
    TalonFXController(Loggable parent_, std::string name, int kCanIdPort) :
          obj(parent_, name), 
            esc_{kCanIdPort},
              configurator_{esc_.GetConfigurator()} {};

    ~TalonFXController() {
      if (gains_helper != nullptr) {
            delete gains_helper;
      }
    };

    ElectronicSpeedController<X>* that() {
      return this;
    }

    int Setup(units::millisecond_t timeout, ControlGainsHelper* gainsHelper,
        bool isInverted = false, IdleMode kIdleMode = kCoast) {

        if (VerifyConnected()) {
          setup = true;
        } else {
          obj.Error("? controller could NOT be setup");
          return -1;
        }
        
        if (kIdleMode == kCoast) esc_.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast); 
        else if (kIdleMode == kBrake) esc_.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

        gains_helper = gainsHelper;

        ctre::configs::VoltageConfigs voltageConfs{};
        ctre::configs::CurrentLimitsConfigs currentConfs{};
        ctre::configs::MotorOutputConfigs motorConfs{};
        ctre::configs::Slot0Configs pidConfs{};


        pidConfs.WithKS(gains_helper->f_.value());
        pidConfs.WithKP(gains_helper->p_.value());
        pidConfs.WithKI(gains_helper->i_.value());
        pidConfs.WithKD(gains_helper->d_.value());

        deviceConfigs.WithSlot0(pidConfs);


        motorConfs.WithInverted(isInverted);

        deviceConfigs.WithMotorOutput(motorConfs);

        currentConfs.SupplyCurrentLimit = gains_helper->current_limit_.value();
        currentConfs.SupplyCurrentThreshold = 50;
        currentConfs.SupplyCurrentLimitEnable = true;

        voltageConfs.PeakForwardVoltage = 12.0;
        voltageConfs.PeakReverseVoltage = -12.0;

        deviceConfigs.WithVoltage(voltageConfs);
        deviceConfigs.WithCurrentLimits(currentConfs);

        pidConfs.WithKS(gains_helper->f_.value());
        pidConfs.WithKP(gains_helper->p_.value());
        pidConfs.WithKI(gains_helper->i_.value());
        pidConfs.WithKD(gains_helper->d_.value());

        deviceConfigs.WithSlot0(pidConfs);

        CheckOk(obj, configurator_.Apply(deviceConfigs));
        
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

        if (kIdleMode == kCoast) esc_.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast); 
        else if (kIdleMode == kBrake) esc_.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

        gains_helper = gainsHelper;

        ctre::configs::VoltageConfigs voltageConfs{};
        ctre::configs::CurrentLimitsConfigs currentConfs{};
        ctre::configs::MotorOutputConfigs motorConfs{};
        ctre::configs::Slot0Configs pidConfs{};


        pidConfs.WithKS(gains_helper->f_.value());
        pidConfs.WithKP(gains_helper->p_.value());
        pidConfs.WithKI(gains_helper->i_.value());
        pidConfs.WithKD(gains_helper->d_.value());

        deviceConfigs.WithSlot0(pidConfs);


        motorConfs.WithInverted(isInverted);

        deviceConfigs.WithMotorOutput(motorConfs);

        currentConfs.SupplyCurrentLimit = gains_helper->current_limit_.value();
        currentConfs.SupplyCurrentThreshold = 50;
        currentConfs.SupplyCurrentLimitEnable = true;

        voltageConfs.PeakForwardVoltage = 12.0;
        voltageConfs.PeakReverseVoltage = -12.0;

        deviceConfigs.WithVoltage(voltageConfs);
        deviceConfigs.WithCurrentLimits(currentConfs);

        pidConfs.WithKS(gains_helper->f_.value());
        pidConfs.WithKP(gains_helper->p_.value());
        pidConfs.WithKI(gains_helper->i_.value());
        pidConfs.WithKD(gains_helper->d_.value());

        deviceConfigs.WithSlot0(pidConfs);

        CheckOk(obj, configurator_.Apply(deviceConfigs));

        return 0;
    };
    
    int Reset(units::millisecond_t timeout) {
      if (!setup) return -1;

      ctre::configs::VoltageConfigs voltageConfs{};
      ctre::configs::CurrentLimitsConfigs currentConfs{};
      ctre::configs::Slot0Configs pidConfs{};

      currentConfs.SupplyCurrentLimit = gains_helper->current_limit_.value();
      currentConfs.SupplyCurrentThreshold = 50;
      currentConfs.SupplyCurrentLimitEnable = true;

      voltageConfs.PeakForwardVoltage = 12.0;
      voltageConfs.PeakReverseVoltage = -12.0;

      deviceConfigs.WithVoltage(voltageConfs);
      deviceConfigs.WithCurrentLimits(currentConfs);

      pidConfs.WithKS(gains_helper->f_.value());
      pidConfs.WithKP(gains_helper->p_.value());
      pidConfs.WithKI(gains_helper->i_.value());
      pidConfs.WithKD(gains_helper->d_.value());

      deviceConfigs.WithSlot0(pidConfs);

      CheckOk(obj, configurator_.Apply(deviceConfigs));
      
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

      // CheckOk(obj, configurator_.Apply(deviceConfigs));
      if (mode == ControlMode::Velocity) {
          ctre::controls::VelocityDutyCycle cntrl{units::turns_per_second_t(conv_.RealToNativeVelocity(output))};
          esc_.SetControl(cntrl);
      }
    };

    void Write(ControlMode mode, X output) {
      if (!setup) return;

      if (esc_.HasResetOccurred()) {
          Reset(CANTimeout);
          esc_.ClearStickyFaults();
      }

      // CheckOk(obj, configurator_.Apply(deviceConfigs));

      if (mode == ControlMode::Position) {
          ctre::controls::PositionDutyCycle cntrl{units::turn_t(conv_.RealToNativePosition(output))};
          esc_.SetControl(cntrl);
      }
    };

    V GetVelocity() {
      if (!setup) throw std::exception();

      return conv_.NativeToRealVelocity(esc_.GetVelocity().GetValueAsDouble());
    };

    X GetPosition() {
      if (!setup) throw std::exception();

      return conv_.NativeToRealPosition(esc_.GetPosition().GetValueAsDouble());
    };

    bool VerifyConnected() {
      return esc_.IsAlive();
    };

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

    util::Converter<X> conv_{util::kTalonPeriod, util::kTalonFXSensorTicks, units::make_unit<X>(1.0)};

    bool setup = false;

    void CheckOk(Loggable& loggable, ctre::phoenix::StatusCode err, std::string field = "TalonFX Config") {
      if (!(err.IsOK() || err.IsWarning())) {
        loggable.Warn("Unable to update {}", field);
      }
    };
};



/*
*
*
*
*/

/*
template <typename X>
class TandemMotorController {
  using V = units::unit_t<units::compound_unit<typename X::unit_type,
                                                units::inverse<units::second>>>;

  private:
  std::vector<ElectronicSpeedController<X>*> esc_list_;
  
  public:
  TandemMotorController() {};
  ~TandemMotorController() {};

  void AddESC(ElectronicSpeedController<X>* esc) {
    esc_list_.push_back(esc);
  }

  X GetAvgPosition() {
    X avg;
    for (auto esc : esc_list_) {
      avg += esc->GetPosition() / ((double) esc_list_.size());
    }
    return avg;
  };

  V GetAvgVelocity() {
    X avg;
    for (auto esc : esc_list_) {
      avg += esc->GetVelocity() / ((double) esc_list_.size());
    }
    return avg;
  };

  bool VerifyAllConnected() {
    for (auto esc : esc_list_) {
      if (!esc->VerifyConnected()) {
        return false;
      }
    }
    return true;
  };

  void SetupAll(ControlGainsHelper* common_gains_helper, std::vector<bool> inversion_list_) {
    if (inversion_list_.size() != esc_list_.size()) {
      return;
    }

    for (int i = 0; i < (int) inversion_list_.size(); i++) {
      esc_list_[i]->Setup(common_gains_helper, inversion_list_[i]);
    }
  }

  void SetupAll(ControlGainsHelper* common_gains_helper, bool invert_all = false) {
    for (ElectronicSpeedController<X>* esc : esc_list_) {
      esc->Setup(common_gains_helper, invert_all);
    }
  }

  void SetupAllConversions(X conv) {
    for (ElectronicSpeedController<X>* esc : esc_list_) {
      esc->SetupConverter(conv);
    }
  }

  void Write(X output) {
    for (auto esc : esc_list_) {
      esc->Write(ControlMode::Position, output);
    }
  }

  void Write(V output) {
    for (auto esc : esc_list_) {
      esc->Write(ControlMode::Velocity, output);
    }
  }

  void Write(units::ampere_t output) {
    for (auto esc : esc_list_) {
      esc->WriteByCurrent(output);
    }
  }

  void Write(double output) {
    for (auto esc : esc_list_) {
      esc->Write(ControlMode::Percent, output);
    }
  }

  void ZeroEncoders(X pos) {
    for (auto esc : esc_list_) {
      esc->ZeroEncoder(pos);
    }
  }

  void ZeroEncoders() {
    for (auto esc : esc_list_) {
      esc->ZeroEncoder();
    }
  }

};*/


template <typename X> 
class RevParallelController {
    using V = units::unit_t<units::compound_unit<typename X::unit_type,
                                                units::inverse<units::second>>>;

    using A = units::ampere_t;

    public:
    RevParallelController(Loggable parent_, std::string name, std::vector<int> kCanIdPorts, 
        rev::CANSparkLowLevel::MotorType motor_type = rev::CANSparkLowLevel::MotorType::kBrushless) :
          obj(parent_, name) {
      for (auto id : kCanIdPorts) {
        auto esc = rev::CANSparkMax(id, motor_type);
        escs_.push_back(&esc);
        obj.Log("Tracking ESC at CANid {}", id);
      }
    };

    ~RevParallelController() {
      if (gains_helper != nullptr) {
            delete gains_helper;
      }
    };

    int Setup(units::millisecond_t timeout, ControlGainsHelper* gainsHelper,
        bool isInverted = false, IdleMode kIdleMode = IdleMode::kCoast) {

         obj.Log("Setting up a REV Parallel");

        if (VerifyConnected()) {
           obj.Log("ALL ESCs connected");
          setup = true;
        } else {
          obj.Error("? controller could NOT be setup");
          return -1;
        }

        for (auto esc_ : escs_) {
          obj.Log("ESC initial setup started");
          if (kIdleMode == kCoast) esc_->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
          else if (kIdleMode == kBrake) esc_->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
          
          esc_->SetCANTimeout(timeout.to<double>());
          esc_->SetInverted(isInverted);
          gains_helper = gainsHelper;

          rev::SparkPIDController pid = esc_->GetPIDController();
          gains_helper->Write(pid, gains_cache_, true);

          esc_->SetSmartCurrentLimit(gainsHelper->current_limit_.value());
          esc_->EnableVoltageCompensation(12.0);
        }

        obj.Log("All ESCs setup ended");
       
        
        return 0;
    };

    int Setup(ControlGainsHelper* gainsHelper, bool isInverted = false,
        IdleMode kIdleMode = IdleMode::kCoast) {

        obj.Log("Setting up a REV Parallel");

        if (VerifyConnected()) {
          setup = true;
          obj.Log("ALL ESCs connected");
        } else {
          obj.Error("? parallel controller could NOT be setup");
          return -1;
        }

        for (auto esc_ : escs_) {
          obj.Log("ESC initial setup started");
          if (kIdleMode == kCoast) esc_->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
          else if (kIdleMode == kBrake) esc_->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
          esc_->SetCANTimeout(CANTimeout.to<double>());
          esc_->SetInverted(isInverted);
        
          rev::SparkPIDController pid = esc_->GetPIDController();
          gains_helper->Write(pid, gains_cache_, true);

          esc_->SetSmartCurrentLimit(gainsHelper->current_limit_.value());
          esc_->EnableVoltageCompensation(12.0);
        }

        obj.Log("All ESCs setup ended");

        return 0;
    };

    
    int Reset(units::millisecond_t timeout) {
      if (!setup) return -1;

      for (auto esc_ : escs_) {

        esc_->SetCANTimeout(timeout.to<double>());

        rev::SparkPIDController pid = esc_->GetPIDController();
        gains_helper->Write(pid, gains_cache_, true);

        esc_->SetSmartCurrentLimit(gains_helper->current_limit_.value());
        esc_->EnableVoltageCompensation(12.0);

      }
      
      return 0;
    };

    void DisableStatusFrames(std::initializer_list<rev::CANSparkLowLevel::PeriodicFrame> frames) {
      if (!setup) return;

      for (auto esc_ : escs_) {
        for (auto f : frames) {
          // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
          auto err = esc_->SetPeriodicFramePeriod(f, 65535);
        }
      }
    };

    void SetupConverter(X conv) {
      if (!setup) return;

      conv_.ChangeConversion(conv);
    };

    void ZeroEncoder(X pos) {
      if (!setup) return;

      for (auto esc_ : escs_) {
        esc_->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42).SetPosition(conv_.RealToNativePosition(pos));
      }
    };

    void ZeroEncoder() {
      if (!setup) return;

      for (auto esc_ : escs_) {
        esc_->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42).SetPosition(0.0);
      }
    };

    void Write(ControlMode mode, double output) {
      if (!setup) return;

      for (auto esc_ : escs_) {
        if (esc_->GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
            Reset(CANTimeout);
            esc_->ClearFaults();
        }

        if (mode == ControlMode::Percent) {

          auto peak_output = gains_helper->peak_output_.value();

          double value = std::max(output, -peak_output);
          value = std::min(value, +peak_output);

          auto pid_controller = esc_->GetPIDController();
          pid_controller.SetReference(value, LocalRevControlMode(mode));
        }
      }
    };

    void Write(ControlMode mode, V output) {
      if (!setup) return;

      for (auto esc_ : escs_) {
        if (esc_->GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
            Reset(CANTimeout);
            esc_->ClearFaults();
        }

        if (mode == ControlMode::Velocity) {
          auto pid_controller = esc_->GetPIDController();
          pid_controller.SetReference(conv_.RealToNativeVelocity(output), LocalRevControlMode(mode));
        }
      }
    };

    void Write(ControlMode mode, X output) {
      if (!setup) return;

      for (auto esc_ : escs_) {
        if (esc_->GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
            Reset(CANTimeout);
            esc_->ClearFaults();
        }

        if (mode == ControlMode::Position) {
          auto pid_controller = esc_->GetPIDController();
          pid_controller.SetReference(conv_.RealToNativePosition(output), LocalRevControlMode(mode));
        }
      }
    };


    void WriteByCurrent(units::ampere_t current) {
      if (!setup) return;

      for (auto esc_ : escs_) {
        if (esc_->GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
            Reset(CANTimeout);
            esc_->ClearFaults();
        }

        auto pid_controller = esc_->GetPIDController();
        pid_controller.SetReference(current.to<double>(), LocalRevControlMode(ControlMode::Current));
      }

    }

    X GetPosition() {
      if (!setup) throw std::exception();

      double sum = 0.0;

      double num_encoders = escs_.size();

      for (auto esc_ : escs_) {
        sum += esc_->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42).GetPosition();
      }

      return conv_.NativeToRealPosition(sum / num_encoders);
    };

    bool VerifyConnected() {
      // GetFirmwareVersion sometimes returns 0 the first time you call it
      obj.Log("Verifying all Connected");
      for (auto esc_ : escs_) {
        obj.Log("Checking ESC");
        esc_->GetFirmwareVersion();
        if (esc_->GetFirmwareVersion() == 0) {
          return false;
        }
      }
      return true;
    };

    void SetInverted(std::vector<bool> invert) {
      if (!setup) return;

      for (int i = 0; i < std::min(invert.size(), escs_.size()); i++) {
        escs_[i]->SetInverted(invert[i]);
      }
    };


    private:
    Loggable obj;
    ControlGainsHelper* gains_helper;

    ControlGains gains_cache_;

    std::vector<rev::CANSparkMax*> escs_;

    util::Converter<X> conv_{util::kSparkMAXPeriod, util::kSparkMAXSensorTicks, units::make_unit<X>(1.0)};

    bool setup = false;

};

}

#endif