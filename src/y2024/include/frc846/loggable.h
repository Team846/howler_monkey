#ifndef FRC846_LOGGABLE_H_
#define FRC846_LOGGABLE_H_

#include <networktables/NetworkTableInstance.h>
#include <units/base.h>

#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <type_traits>
#include <variant>

#include "frc/DataLogManager.h"
#include "util/logger.h"

namespace frc846 {
class Loggable {
 public:
  Loggable(const Loggable& parent_, std::string name)
      : name_{parent_.name() + "/" + name}, logger(name_) {
    Loggable(parent_.name() + "/" + name);
  }

  Loggable(std::string name) : name_{name}, logger(name_) {}

  std::string name() const { return name_; }

  template <typename... T>
  void Log(fmt::format_string<T...> fmt, T&&... args) const {
    logger.Log(name_, fmt, std::forward<T>(args)...);
  }
  template <typename... T>
  void Warn(fmt::format_string<T...> fmt, T&&... args) {
    logger.Warn(name_, fmt, std::forward<T>(args)...);
    warn_count_++;
  }
  template <typename... T>
  void Error(fmt::format_string<T...> fmt, T&&... args) {
    logger.Error(name_, fmt, std::forward<T>(args)...);
    error_count_++;
  }

  static unsigned int GetWarnCount();
  static unsigned int GetErrorCount();

  static std::string Join(std::string p, std::string n);

 private:
  const std::string name_;

  static unsigned int warn_count_;
  static unsigned int error_count_;

  frc846::Logger logger;
};
}  // namespace frc846

#endif