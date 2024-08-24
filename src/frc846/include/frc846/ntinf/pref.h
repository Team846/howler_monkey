#pragma once

#include <frc/Preferences.h>
#include <networktables/NetworkTableInstance.h>
#include <units/base.h>

#include <type_traits>

#include "frc846/base/loggable.h"
#include "frc846/ntinf/fstore.h"

namespace frc846::ntinf {

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
  Pref(const frc846::base::Loggable& parent, std::string name, T fallback);
  Pref(const frc846::base::Loggable& parent, std::string name);

  ~Pref() { pref_table_->RemoveListener(entry_listener_); }

 private:
  Pref(const frc846::base::Loggable& parent, std::string name, T fallback,
       std::function<void(std::string, T, frc846::ntinf::FPointer)> init,
       std::function<void(frc846::ntinf::FPointer, T)> set,
       std::function<T(std::string, T)> get)
      : f_ptr_{frc846::base::Loggable::Join(parent.name(), name)} {
    FunkyStore::FP_HardReadPrefs();
    // Full networktables key (parent name + pref name)
    auto full_key = frc846::base::Loggable::Join(parent.name(), name);

    // If the entry already exists, get its value. Otherwise, create the entry.
    init(full_key, fallback, f_ptr_);
    value_ = get(full_key, fallback);

    if (value_ != fallback) {
      parent.Log("Definition of preference `{}` is outdated.", full_key);
    }

    // Update pref when preference is new or updated.
    entry_listener_ = pref_table_->AddListener(
        full_key, NT_EVENT_PROPERTIES | NT_EVENT_PUBLISH | NT_EVENT_VALUE_ALL,
        [=, this]([[maybe_unused]] auto&&... unused) {
          if (value_ != get(full_key, fallback)) {
            value_ = get(full_key, fallback);
            parent.Log("`{}` updated to {}", name, value_);
            set(f_ptr_, value_);
          }
        });
  }

 public:
  // Access the preference value.
  T value() const { return value_; }

 private:
  T value_;

  NT_Listener entry_listener_;

  frc846::ntinf::FPointer f_ptr_;

  std::shared_ptr<nt::NetworkTable> pref_table_ =
      nt::NetworkTableInstance::GetDefault().GetTable("Preferences");
};

template <class U>
Pref<U>::Pref(const frc846::base::Loggable& parent, std::string name,
              U fallback)
    : Pref{parent,
           fmt::format("{} ({})", name,
                       units::abbreviation(units::make_unit<U>(0))),
           fallback,
           [](std::string name, U fallback, frc846::ntinf::FPointer ptr) {
             frc::Preferences::SetDouble(
                 name, ptr.double_value(fallback.template to<double>()));
           },
           [](frc846::ntinf::FPointer ptr, U val) {
             ptr.set(val.template to<double>());
           },
           [](std::string name, U fallback) {
             return units::make_unit<U>(frc::Preferences::GetDouble(
                 name, fallback.template to<double>()));
           }} {
  static_assert(units::traits::is_unit_t<U>(), "must be a unit");
}

template <class U>
Pref<U>::Pref(const frc846::base::Loggable& parent, std::string name)
    : Pref{parent, name, units::abbreviation(units::make_unit<U>(0))} {}

template <>
inline Pref<bool>::Pref(const frc846::base::Loggable& parent, std::string name,
                        bool fallback)
    : Pref<bool>{
          parent,
          name,
          fallback,
          [](std::string name, bool fallback, frc846::ntinf::FPointer ptr) {
            frc::Preferences::SetBoolean(name, ptr.bool_value(fallback));
          },
          [](frc846::ntinf::FPointer ptr, bool val) { ptr.set(val); },
          [](std::string name, bool fallback) {
            return frc::Preferences::GetBoolean(name, fallback);
          },
      } {}

template <>
inline Pref<bool>::Pref(const frc846::base::Loggable& parent, std::string name)
    : Pref<bool>{parent, name, false} {}

template <>
inline Pref<double>::Pref(const frc846::base::Loggable& parent,
                          std::string name, double fallback)
    : Pref<double>{
          parent,
          name,
          fallback,
          [](std::string name, double fallback, frc846::ntinf::FPointer ptr) {
            frc::Preferences::SetDouble(name, ptr.double_value(fallback));
          },
          [](frc846::ntinf::FPointer ptr, double val) { ptr.set(val); },
          [](std::string name, double fallback) {
            return frc::Preferences::GetDouble(name, fallback);
          },
      } {}

template <>
inline Pref<double>::Pref(const frc846::base::Loggable& parent,
                          std::string name)
    : Pref<double>{parent, name, 0.0} {}

template <>
inline Pref<int>::Pref(const frc846::base::Loggable& parent, std::string name,
                       int fallback)
    : Pref<int>{
          parent,
          name,
          fallback,
          [](std::string name, int fallback, frc846::ntinf::FPointer ptr) {
            frc::Preferences::SetInt(name, ptr.int_value(fallback));
          },
          [](frc846::ntinf::FPointer ptr, int val) { ptr.set(val); },
          [](std::string name, int fallback) {
            return frc::Preferences::GetInt(name, fallback);
          },
      } {}

template <>
inline Pref<int>::Pref(const frc846::base::Loggable& parent, std::string name)
    : Pref<int>{parent, name, 0} {}

template <>
inline Pref<std::string>::Pref(const frc846::base::Loggable& parent,
                               std::string name, std::string fallback)
    : Pref<std::string>{
          parent,
          name,
          fallback,
          [](std::string name, std::string fallback,
             frc846::ntinf::FPointer ptr) {
            frc::Preferences::SetString(name, ptr.string_value(fallback));
          },
          [](frc846::ntinf::FPointer ptr, std::string val) { ptr.set(val); },
          [](std::string name, std::string fallback) {
            return frc::Preferences::GetString(name, fallback);
          },
      } {}

// TODO: ADD Preference layers to nest gains within different layers
// Subsystem, Event loop, gains, safeties, offloaded output vals, etc

}  // namespace frc846::ntinf
