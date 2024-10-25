#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/base.h>

#include <any>
#include <string>
#include <unordered_map>

#include "frc846/base/Loggable.h"
#include "frc846/ntinf/grapher.h"

namespace frc846::robot {

class RSTable : frc846::base::Loggable {
 private:
  std::unordered_map<std::string, std::any> table_;

 public:
  RSTable(std::string table_name);
  virtual ~RSTable() = default;

  template <typename T>
  void Set(std::string key, T value) {
    table_[key] = value;

    if (units::traits::is_unit_t<T>()) {
      frc::SmartDashboard::PutData(name(), value.template to<double>());
    } else if (std::is_same_v<T, bool>) {
      frc::SmartDashboard::PutBoolean(name(), value);
    } else if (std::is_same_v<T, std::string>) {
      frc::SmartDashboard::PutString(name(), value);
    } else if (std::is_same_v<T, int>) {
      frc::SmartDashboard::PutNumber(name(), value);
    } else if (std::is_same_v<T, double>) {
      frc::SmartDashboard::PutNumber(name(), value);
    } else {
      Warn("Unable to graph RSTable entry of type {} with key {}.",
           typeid(T).name(), key);
    }
  }

  template <typename T>
  T Get(std::string key) {
    try {
      return std::any_cast<T>(table_[key]);
    } catch (const std::bad_any_cast& exc) {
      Error("Incorrect type specified when accessing RSTable key {}.", key);
    } catch (const std::out_of_range& exc) {
      Warn("Key {} not found in RSTable.", key);
    }
    return T{};
  }

  std::vector<std::string> ListKeys();
};

class RobotState {
  static std::unordered_map<std::string, RSTable*> tables_;

 public:
  static RSTable* getTable(std::string table_name);
};

};  // namespace frc846::robot