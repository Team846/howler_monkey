#ifndef FRC846_MOTION_H_
#define FRC846_MOTION_H_

#include <units/angle.h>
#include <units/length.h>

#include "frc846/util/pref.h"
#include "frc846/util/share_tables.h"

namespace frc846::motion {

class MotionSnapshot {
 private:
  double pos_;

 public:
  MotionSnapshot(units::degree_t pos) { pos_ = pos.to<double>(); }

  MotionSnapshot(units::inch_t pos) { pos_ = pos.to<double>(); }

  MotionSnapshot(double pos) { pos_ = pos; }

  double value() { return pos_; }
};

class MotionTarget {
 public:
  static frc846::Loggable preferences_loggable;

 private:
  const frc846::Pref<double> pos_pref_;

  bool isAngularType;

 public:
  MotionTarget(std::string pname, std::string name, units::inch_t pos)
      : pos_pref_{preferences_loggable, Loggable::Join(pname, name),
                  pos.to<double>()},
        isAngularType{false} {}

  MotionTarget(std::string pname, std::string name, units::degree_t pos)
      : pos_pref_{preferences_loggable, Loggable::Join(pname, name),
                  pos.to<double>()},
        isAngularType{true} {}

  double value() { return pos_pref_.value(); }

  bool isAngular() { return isAngularType; }
};

class MotionProfile {
 public:
  MotionProfile(std::vector<frc846::util::SharePointer> ref_ptrs,
                std::vector<MotionTarget> targets, std::vector<int> exponents)
      : ref_ptrs_{ref_ptrs}, targets_{targets}, exponents_{exponents} {}

  //  std::vector<MotionSnapshot> GenerateMotionSnapshots() {
  //     std::vector<MotionSnapshot> snapshots;
  //     for (auto& ptr : ref_ptrs_) {
  //         snapshots.push_back(MotionSnapshot(ptr.value()));
  //     }
  //     return snapshots;
  //  }

  double getFinalTarget(int index) { return targets_[index].value(); }

 private:
  std::vector<frc846::util::SharePointer>& ref_ptrs_;
  std::vector<MotionTarget>& targets_;
  std::vector<int>& exponents_;
};

}  // namespace frc846::motion

#endif  // FRC846_MOTION_H_