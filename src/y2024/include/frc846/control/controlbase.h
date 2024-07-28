#pragma once

#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <units/current.h>

#include <algorithm>
#include <chrono>
#include <ctre/phoenix6/TalonFX.hpp>
#include <initializer_list>
#include <thread>
#include <variant>

#include "frc/RobotController.h"
#include "frc846/base/loggable.h"
#include "frc846/control/config.h"
#include "frc846/ctre_namespace.h"
#include "frc846/util/conversions.h"

FRC846_CTRE_NAMESPACE()

namespace frc846::control {

struct DNC {
  static constexpr double CANTimeout = 20.0;  // ms

  static constexpr int numRetries = 5;
  static constexpr int maxQWaitLoops = 50;

  static constexpr int QTimeWait = 100;  // ms

  static constexpr double QMaxCANUtil = 0.6;  // frac
};

class ErrorParsing {
 public:
  static unsigned int getErrorCode(rev::REVLibError err) {
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

  static unsigned int getErrorCode(ctre::phoenix::StatusCode& err) {
    if (err.IsOK()) {
      return 0;
    } else if (err.IsWarning()) {
      return 11;
    } else if (err.IsError()) {
      return 1;
    }

    return -1;
  }

  static std::string parseErrorCode(unsigned int err) {
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
  bool check(unsigned int messageType, double messageValue) {
    if (lastFailure) return true;

    if (messageType != lastMessageType ||
        std::abs(messageValue - lastMessageValue) > 0.005) {
      lastMessageType = messageType;
      lastMessageValue = messageValue;
      return true;
    }

    return false;
  }

  void registerSuccess(bool success) { lastFailure = !success; }

 private:
  unsigned int lastMessageType;
  double lastMessageValue;
  bool lastFailure = false;
};

/*
 * Common class for all custom ESC wrappers
 */

template <typename X>
class BaseESC {
  using V = units::unit_t<
      units::compound_unit<typename X::unit_type,  // the velocity unit is equal
                                                   // to the position unit / 1_s
                           units::inverse<units::second>>>;

  using A = units::ampere_t;  // current

 public:
  // virtual void OverrideInvert(
  //     bool invert) = 0;  // for systems where using same config helper for 2+
  //                        // motors but 1+ of those motors is inverted. ex.
  //                        2024
  //                        // pivot

  virtual void SetVoltageCompensationAuton(
      bool auton) = 0;  // to switch between voltage compensation presets for
                        // autonomous and teleop

  virtual void ZeroEncoder(X pos) = 0;

  virtual void WriteDC(double output) = 0;

  virtual void WriteVelocity(V output) = 0;

  virtual void WritePosition(X output) = 0;

  virtual V GetVelocity() = 0;

  virtual double GetVelocityPercentage() = 0;

  virtual X GetPosition() = 0;

  virtual bool VerifyConnected() = 0;

  virtual bool GetInverted() = 0;

  virtual units::ampere_t GetCurrent() = 0;

 protected:
  void Q(frc846::base::Loggable& lg, std::function<bool()> f) {
    int numLoops = 0;
    do {
      numLoops++;
      std::this_thread::sleep_for(std::chrono::milliseconds(DNC::QTimeWait));

      if (numLoops >= DNC::maxQWaitLoops) {
        lg.Warn("Exited Config Q Early. Continuously high CAN bus usage.");
        break;
      }
    } while (frc::RobotController::GetCANStatus().percentBusUtilization >=
             DNC::QMaxCANUtil);

    for (int i = 0; i < DNC::numRetries; i++) {
      if (f()) {
        if (i != 0) {
          lg.Warn("Config required {} retries.", i + 1);
        }
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    lg.Error("Configuration may not have been set. Errored {}/{} times.",
             DNC::numRetries, DNC::numRetries);
  }
};

};  // namespace frc846::control