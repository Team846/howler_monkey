#pragma once

#include <fmt/core.h>
#include <frc/Timer.h>
#include <networktables/NetworkTableInstance.h>
#include <units/base.h>

#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <type_traits>
#include <variant>

#include "frc/DataLogManager.h"

namespace frc846 {

class Logger {
 private:
  std::string caller;
  bool w_record;

  // std::string_view last_log;
  // double last_timestamp;

  // std::shared_ptr<nt::NetworkTable> table_;

 public:
  Logger(std::string t_caller) {
    caller = t_caller;
    w_record = true;
  }

  float twodp(float num) const {
    float value = (int)(num * 100 + 0.5);
    return (float)value / 100;
  }

  template <typename... T>
  void Log(std::string level, fmt::format_string<T...> fmt, T&&... args) const {
    // last_timestamp = frc::Timer::GetFPGATimestamp().to<double>();

    auto t_log =
        "LOG [" + level + "] (" +
        fmt::format("{}", twodp(frc::Timer::GetFPGATimestamp().to<double>())) +
        ", " +
        fmt::format("{}", twodp(frc::Timer::GetMatchTime().to<double>())) +
        ") " +
        fmt::format(std::forward<fmt::format_string<T...>>(fmt),
                    std::forward<T>(args)...);

    // last_log = t_log;

    frc::DataLogManager::Log(t_log);

    auto filename = "/home/lvuser/" + caller + ".log";

    // if (w_record) {
    //   if (!std::filesystem::exists(filename) ||
    //       std::filesystem::file_size(filename) > 1000000) {
    //     std::ofstream t_file(filename, std::fstream::in | std::fstream::out |
    //                                        std::fstream::trunc);
    //     t_file << "Logger Overwrite "
    //            << (fmt::format(
    //                   "{}",
    //                   twodp(frc::Timer::GetFPGATimestamp().to<double>())))
    //            << "\n";
    //     t_file << "Logger " << t_log << "\n";
    //     t_file.close();

    //   } else {
    //     std::ofstream t_file(
    //         filename, std::fstream::in | std::fstream::out |
    //         std::fstream::app);
    //     t_file << "Logger " << t_log << "\n";
    //     t_file.close();
    //   }
    // }
  }

  template <typename... T>
  void Warn(std::string level, fmt::format_string<T...> fmt,
            T&&... args) const {
    // last_timestamp = frc::Timer::GetFPGATimestamp().to<double>();

    auto t_log =
        "WARN [" + level + "] (" +
        fmt::format("{}", twodp(frc::Timer::GetFPGATimestamp().to<double>())) +
        ", " +
        fmt::format("{}", twodp(frc::Timer::GetMatchTime().to<double>())) +
        ") " +
        fmt::format(std::forward<fmt::format_string<T...>>(fmt),
                    std::forward<T>(args)...);

    // last_log = t_log;

    frc::DataLogManager::Log(t_log);

    auto filename = "/home/lvuser/" + caller + ".log";

    // if (w_record) {
    //   if (!std::filesystem::exists(filename) ||
    //       std::filesystem::file_size(filename) > 1000000) {
    //     std::ofstream t_file(filename, std::fstream::in | std::fstream::out |
    //                                        std::fstream::trunc);
    //     t_file << "Logger Overwrite "
    //            << fmt::format(
    //                   "{}",
    //                   twodp(frc::Timer::GetFPGATimestamp().to<double>()))
    //            << "\n";
    //     t_file << "Logger " << t_log << "\n";
    //     t_file.close();

    //   } else {
    //     std::ofstream t_file(
    //         filename, std::fstream::in | std::fstream::out |
    //         std::fstream::app);
    //     t_file << "Logger " << t_log << "\n";
    //     t_file.close();
    //   }
    // }
  }

  template <typename... T>
  void Error(std::string level, fmt::format_string<T...> fmt,
             T&&... args) const {
    // last_timestamp = frc::Timer::GetFPGATimestamp().to<double>();

    auto t_log =
        "ERROR [" + level + "] (" +
        fmt::format("{}", twodp(frc::Timer::GetFPGATimestamp().to<double>())) +
        ", " +
        fmt::format("{}", twodp(frc::Timer::GetMatchTime().to<double>())) +
        ") " +
        fmt::format(std::forward<fmt::format_string<T...>>(fmt),
                    std::forward<T>(args)...);

    // last_log = t_log;

    frc::DataLogManager::Log(t_log);

    auto filename = "/home/lvuser/" + caller + ".log";

    // if (w_record) {
    //   if (!std::filesystem::exists(filename) ||
    //       std::filesystem::file_size(filename) > 1000000) {
    //     std::ofstream t_file(filename, std::fstream::in | std::fstream::out |
    //                                        std::fstream::trunc);
    //     t_file << "Logger Overwrite "
    //            << fmt::format(
    //                   "{}",
    //                   twodp(frc::Timer::GetFPGATimestamp().to<double>()))
    //            << "\n";
    //     t_file << "Logger " << t_log << "\n";
    //     t_file.close();

    //   } else {
    //     std::ofstream t_file(
    //         filename, std::fstream::in | std::fstream::out |
    //         std::fstream::app);
    //     t_file << "Logger " << t_log << "\n";
    //     t_file.close();
    //   }
    // }
  }

  /*
  std::string_view retrieveLastLog() {
    return last_log;
  }

  double timeOfLastLog() {
    return last_timestamp;
  }
  */
};

}  // namespace frc846
