#ifndef frcLib846_LOGGER_H_
#define frcLib846_LOGGER_H_

#include <networktables/NetworkTableInstance.h>
#include <units/base.h>

#include <type_traits>
#include <variant>

#include <frc/Timer.h>
#include "frc/DataLogManager.h"
#include <fmt/core.h>

#include <initializer_list>
#include <iostream>
#include <fstream>
#include <filesystem>

namespace frcLib846 {

  class Logger {
    private:
      std::string caller;
      bool w_record;

      // std::string_view last_log;
      // double last_timestamp;

      //std::shared_ptr<nt::NetworkTable> table_;

    public:
      Logger(std::string t_caller) {
        caller = t_caller;
        w_record = true;
      }

      template <typename... T>
      void Log(std::string level, fmt::format_string<T...> fmt, T&&... args) const {
        //last_timestamp = frc::Timer::GetFPGATimestamp().to<double>();

        auto t_log = "LOG [" + level + "] (" + fmt::format("{}", frc::Timer::GetFPGATimestamp().to<double>()) + ") " + fmt::format(std::forward<fmt::format_string<T...>>(fmt),
                           std::forward<T>(args)...);

        //last_log = t_log;

        frc::DataLogManager::Log(t_log);


        auto filename = "/home/lvuser/"+caller+".log";

        if (w_record) {
          if (!std::filesystem::exists(filename) || std::filesystem::file_size(filename) > 1000000)  {

            std::ofstream t_file(filename,  std::fstream::in | std::fstream::out | std::fstream::trunc);
            t_file << "Logger Overwrite " << fmt::format("{}", frc::Timer::GetFPGATimestamp().to<double>()) << "\n";
            t_file << "Logger " << t_log << "\n";
            t_file.close();

          } else {
            std::ofstream t_file(filename, std::fstream::in | std::fstream::out | std::fstream::app); 
            t_file << "Logger " << t_log << "\n";
            t_file.close();
          }
        }
      }

      template <typename... T>
      void Warn(std::string level, fmt::format_string<T...> fmt, T&&... args) const {
        //last_timestamp = frc::Timer::GetFPGATimestamp().to<double>();

        auto t_log = "WARN [" + level + "] (" + fmt::format("{}", frc::Timer::GetFPGATimestamp().to<double>()) + ") " + fmt::format(std::forward<fmt::format_string<T...>>(fmt),
                           std::forward<T>(args)...);

        //last_log = t_log;

        frc::DataLogManager::Log(t_log);

        auto filename = "/home/lvuser/"+caller+".log";

        if (w_record) {
          if (!std::filesystem::exists(filename) || std::filesystem::file_size(filename) > 1000000) {

            std::ofstream t_file(filename,  std::fstream::in | std::fstream::out | std::fstream::trunc);
            t_file << "Logger Overwrite " << fmt::format("{}", frc::Timer::GetFPGATimestamp().to<double>()) << "\n";
            t_file << "Logger " << t_log << "\n";
            t_file.close();

          } else {
            std::ofstream t_file(filename, std::fstream::in | std::fstream::out | std::fstream::app); 
            t_file << "Logger " << t_log << "\n";
            t_file.close();
          }
        }
      }

      template <typename... T>
      void Error(std::string level, fmt::format_string<T...> fmt, T&&... args) const {
        //last_timestamp = frc::Timer::GetFPGATimestamp().to<double>();

        auto t_log = "ERROR [" + level + "] (" + fmt::format("{}", frc::Timer::GetFPGATimestamp().to<double>()) + ") " + fmt::format(std::forward<fmt::format_string<T...>>(fmt),
                           std::forward<T>(args)...);

        //last_log = t_log;

        frc::DataLogManager::Log(t_log);


        auto filename = "/home/lvuser/"+caller+".log";

        if (w_record) {
          if (!std::filesystem::exists(filename) || std::filesystem::file_size(filename) > 1000000) {

            std::ofstream t_file(filename,  std::fstream::in | std::fstream::out | std::fstream::trunc);
            t_file << "Logger Overwrite " << fmt::format("{}", frc::Timer::GetFPGATimestamp().to<double>()) << "\n";
            t_file << "Logger " << t_log << "\n";
            t_file.close();

          } else {
            std::ofstream t_file(filename, std::fstream::in | std::fstream::out | std::fstream::app); 
            t_file << "Logger " << t_log << "\n";
            t_file.close();
          }
        }
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

}  // namespace frcLib846

#endif  // frcLib846_LOGGER_H_