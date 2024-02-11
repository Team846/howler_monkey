#ifndef FRC846_PREF_H_
#define FRC846_PREF_H_

#include <networktables/NetworkTableInstance.h>
#include <units/base.h>

#include <type_traits>

#include "frc/Preferences.h"
#include "frc846/loggable.h"

namespace frc846 {

// A hot-configurable preference.
//
// Prefs are created in the Preferences table in NetworkTables and
// automatically update when the table entry is edited to make a runtime
// editable value.
template <class T>
class Pref {
 public:
  // Construct a new pref for a unit type with a fallback value.
  //
  // The pref name will be post-fixed with the unit (e.g. `length (in)`).
  Pref(const frc846::Loggable& parent, std::string name, T fallback);
  Pref(const frc846::Loggable& parent, std::string name);


  ~Pref() { pref_table_->RemoveListener(entry_listener_); }

 private:
  Pref(const frc846::Loggable& parent, std::string name, T fallback,
       std::function<void(std::string, T)> init,
       std::function<T(std::string, T)> get) {
    // Full networktables key (parent name + pref name)
    auto full_key = Loggable::Join(parent.name(), name);

    // If the entry already exists, get its value. Otherwise, create the entry.
    if (frc::Preferences::ContainsKey(full_key)) {
      value_ = get(full_key, fallback);
    } else {
      parent.Log("`{}` initializing to fallback {}", name, fallback);
      init(full_key, fallback);
      value_ = fallback;
    }

    // Update pref when preference is new or updated.
    entry_listener_ = pref_table_->AddListener(
        full_key, NT_EVENT_PROPERTIES | NT_EVENT_PUBLISH | NT_EVENT_VALUE_ALL,
        [=, this]([[maybe_unused]] auto&&... unused) {
          value_ = get(full_key, fallback);
          parent.Log("`{}` updated to {}", name, value_);
        });
  }

 public:
  // Access the preference value.
  T value() const { return value_; }

 private:
  T value_;

  NT_Listener entry_listener_;

  std::shared_ptr<nt::NetworkTable> pref_table_ =
      nt::NetworkTableInstance::GetDefault().GetTable("Preferences");
};

template <class U>
Pref<U>::Pref(const frc846::Loggable& parent, std::string name, U fallback)
    : Pref{parent,
           fmt::format("{} ({})", name,
                       units::abbreviation(units::make_unit<U>(0))),
           fallback,
           [](std::string name, U fallback) {
             frc::Preferences::SetDouble(name, fallback.template to<double>());
           },
           [](std::string name, U fallback) {
             return units::make_unit<U>(frc::Preferences::GetDouble(
                 name, fallback.template to<double>()));
           }} {
  static_assert(units::traits::is_unit_t<U>(), "must be a unit");
}

template <class U>
Pref<U>::Pref(const frc846::Loggable& parent, std::string name)
    : Pref{parent,
           fmt::format("{} ({})", name,
                       units::abbreviation(units::make_unit<U>(0))),
           units::make_unit<U>(0),
           [](std::string name, U fallback) {
             frc::Preferences::SetDouble(name, fallback.template to<double>());
           },
           [](std::string name, U fallback) {
             return units::make_unit<U>(frc::Preferences::GetDouble(
                 name, fallback.template to<double>()));
           }} {
  static_assert(units::traits::is_unit_t<U>(), "must be a unit");
}


template <>
inline Pref<bool>::Pref(const frc846::Loggable& parent, std::string name,
                        bool fallback)
    : Pref<bool>{
          parent,
          name,
          fallback,
          [](std::string name, bool fallback) {
            frc::Preferences::SetBoolean(name, fallback);
          },
          [](std::string name, bool fallback) {
            return frc::Preferences::GetBoolean(name, fallback);
          },
      } {}

template <>
inline Pref<bool>::Pref(const frc846::Loggable& parent, std::string name)
    : Pref<bool>{
          parent,
          name,
          false,
          [](std::string name, bool fallback) {
            frc::Preferences::SetBoolean(name, fallback);
          },
          [](std::string name, bool fallback) {
            return frc::Preferences::GetBoolean(name, fallback);
          },
      } {}

template <>
inline Pref<double>::Pref(const frc846::Loggable& parent, std::string name,
                          double fallback)
    : Pref<double>{
          parent,
          name,
          fallback,
          [](std::string name, double fallback) {
            frc::Preferences::SetDouble(name, fallback);
          },
          [](std::string name, double fallback) {
            return frc::Preferences::GetDouble(name, fallback);
          },
      } {}

template <>
inline Pref<double>::Pref(const frc846::Loggable& parent, std::string name)
    : Pref<double>{
          parent,
          name,
          0.0,
          [](std::string name, double fallback) {
            frc::Preferences::SetDouble(name, fallback);
          },
          [](std::string name, double fallback) {
            return frc::Preferences::GetDouble(name, fallback);
          },
      } {}

template <>
inline Pref<int>::Pref(const frc846::Loggable& parent, std::string name,
                       int fallback)
    : Pref<int>{
          parent,
          name,
          fallback,
          [](std::string name, int fallback) {
            frc::Preferences::SetInt(name, fallback);
          },
          [](std::string name, int fallback) {
            return frc::Preferences::GetInt(name, fallback);
          },
      } {}

template <>
inline Pref<int>::Pref(const frc846::Loggable& parent, std::string name)
    : Pref<int>{
          parent,
          name,
          0,
          [](std::string name, int fallback) {
            frc::Preferences::SetInt(name, fallback);
          },
          [](std::string name, int fallback) {
            return frc::Preferences::GetInt(name, fallback);
          },
      } {}

template <>
inline Pref<std::string>::Pref(const frc846::Loggable& parent, std::string name,
                               std::string fallback)
    : Pref<std::string>{
          parent,
          name,
          fallback,
          [](std::string name, std::string fallback) {
            frc::Preferences::SetString(name, fallback);
          },
          [](std::string name, std::string fallback) {
            return frc::Preferences::GetString(name, fallback);
          },
      } {}

// TODO: ADD Preference layers to nest gains within different layers
// Subsystem, Event loop, gains, safeties, offloaded output vals, etc

}  // namespace frc846

#endif  // FRC846_PREF_H_