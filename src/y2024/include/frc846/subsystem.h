#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>

#include "frc846/loggable.h"

namespace frc846 {

#define FRC846_VERIFY(expr, ok, fail_msg)    \
  do {                                       \
    if (!(expr)) {                           \
      ok = false;                            \
      Error("HARDWARE ERROR: {}", fail_msg); \
    }                                        \
  } while (0)

// Non-templated subsystem base class.
class SubsystemBase : public Loggable {
 public:
  SubsystemBase(std::string name) : Loggable{name} {}
  SubsystemBase(Loggable parent, std::string name) : Loggable{parent, name} {}

  virtual ~SubsystemBase() = default;

  virtual void UpdateReadings() = 0;
  virtual void UpdateHardware() = 0;
  virtual bool VerifyHardware() = 0;
  virtual void SetTargetZero() = 0;
};

// Base class for robot subsystems.
template <class Readings, class Target>
class Subsystem : public frc2::SubsystemBase, public SubsystemBase {
 public:
  // Construct a new subsystem.
  explicit Subsystem(std::string name, bool init = false)
      : frc846::SubsystemBase{name}, init_{init} {
    if (init_) Init();
  }

  // Construct a subsystem as a child of another subsystem.
  Subsystem(const Loggable& parent, std::string name, bool init = false)
      : frc846::SubsystemBase{parent, name}, init_{init} {
    if (init_) Init();
  }

  void enable() {
    init_ = true;
    if (!initialized) Init();
  }

  void disable() { init_ = false; }
  bool is_initialized() { return init_; }

  Subsystem(const Subsystem&) = delete;
  Subsystem& operator=(const Subsystem&) = delete;

  virtual ~Subsystem() = default;

 private:
  // Common constructor.
  void Init() {
    SetName(name());
    Log("Initializing");
    initialized = true;
  }

  bool init_;
  bool initialized;

 public:
  // Get the zero state target.
  virtual Target ZeroTarget() const = 0;

  // Fetches new readings and update subsystem readings state.
  void UpdateReadings() override {
    if (init_) {
      readings_ = GetNewReadings();
    } else {
      readings_ = Readings{};
    }
  }

  // Writes to subsystem hardware with the latest target output.
  void UpdateHardware() override {
    if (init_) DirectWrite(target_);
  }

  virtual bool VerifyHardware() override = 0;

  // Get the latest readings.
  Readings readings() const { return readings_; };

  // Set the subystem target state.
  void SetTarget(Target target) { target_ = target; }

  // Set the subsystem to its zero state.
  void SetTargetZero() override { target_ = ZeroTarget(); }

  auto GetTarget() { return target_; }

  void WriteToHardware(Target target) {
    if (init_) {
      SetTarget(target);
      UpdateHardware();
    }
  }

 protected:
  Readings readings_;
  Target target_;

 private:
  // Fetches and return new readings.
  virtual Readings GetNewReadings() = 0;

  // Writes output to hardware.
  virtual void DirectWrite(Target target) = 0;
};

}  // namespace frc846
