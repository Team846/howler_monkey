#pragma once

#include <fmt/format.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "frc846/util/pref.h"

namespace frc846 {

// Log a value to smart dashboard.
//
// TODO add graphing to file
template <class T>
class Grapher : public Loggable {
 public:
  Grapher(const Loggable& parent, std::string name);

 private:
  Grapher(const Loggable& parent, std::string name,
          std::function<void(std::string_view, T)> put_nt)
      : Loggable{parent, fmt::format("{}_graph", name)},
        full_key_(Loggable::Join(parent.name(), name)),
        put_nt_(put_nt) {}

 public:
  void Graph(T value) { put_nt_(full_key_, value); }

 private:
  std::string full_key_;
  std::function<void(std::string_view, T)> put_nt_;
};

template <class U>
Grapher<U>::Grapher(const Loggable& parent, std::string name)
    : Grapher{
          parent,
          fmt::format("{} ({})", name,
                      units::abbreviation(units::make_unit<U>(0))),
          [](std::string_view key, U value) {
            frc::SmartDashboard::PutNumber(key, value.template to<double>());
          },
      } {
  static_assert(units::traits::is_unit_t<U>(), "must be a unit");
}

template <>
inline Grapher<bool>::Grapher(const Loggable& parent, std::string name)
    : Grapher{parent, name, frc::SmartDashboard::PutBoolean} {}

template <>
inline Grapher<double>::Grapher(const Loggable& parent, std::string name)
    : Grapher{parent, name, frc::SmartDashboard::PutNumber} {}

template <>
inline Grapher<int>::Grapher(const Loggable& parent, std::string name)
    : Grapher{parent, name, frc::SmartDashboard::PutNumber} {}

template <>
inline Grapher<std::string>::Grapher(const Loggable& parent, std::string name)
    : Grapher{parent, name, frc::SmartDashboard::PutString} {}
}  // namespace frc846
