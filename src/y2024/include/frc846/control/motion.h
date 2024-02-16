#ifndef FRC846_MOTION_H_
#define FRC846_MOTION_H_

#include "frc846/util/pref.h"
#include "frc846/util/share_tables.h"
#include <units/angle.h>
#include <units/length.h>


namespace frc846::motion {

class MotionSnapshot {
    private:
    double pos_;

    public:
    MotionSnapshot(units::degree_t pos) {
        pos_ = pos.to<double>();
    }

    MotionSnapshot(units::inch_t pos) {
        pos_ = pos.to<double>();
    }

    MotionSnapshot(double pos) {
        pos_ = pos;
    }

    double value() {
        return pos_;
    }
};

class MotionTarget {
    static frc846::Loggable preferences_loggable;

    private:
    frc846::Pref<double> pos_pref_;

    bool isAngularType;

    public:
    MotionTarget(std::string pname, std::string name, units::inch_t pos) :
        pos_pref_{preferences_loggable, Loggable::Join(pname, name), pos.to<double>()} {
        isAngularType = false;
    }

    MotionTarget(std::string pname, std::string name, units::degree_t pos) :
        pos_pref_{preferences_loggable, Loggable::Join(pname, name), pos.to<double>()} {
        isAngularType = true;
    }

    double value() {
        return pos_pref_.value();
    }

    bool isAngular() {
        return isAngularType;
    }
};

class MotionProfile {
 public:

 MotionProfile(std::vector<frc846::util::SharePointer> ref_ptrs, 
    std::vector<MotionTarget> targets, std::vector<int> exponents) : ref_ptrs_{ref_ptrs},
        targets_{targets}, exponents_{exponents} {}

 std::vector<MotionSnapshot> GenerateMotionSnapshots() {
    std::vector<MotionSnapshot> snapshots;
    for (auto& ptr : ref_ptrs_) {
        snapshots.push_back(MotionSnapshot(ptr.value()));
    }
    return snapshots;
 }

 double getFinalTarget(int index) {
    return targets_[index].value();
 }

 double calculateNextTarget(int index, std::vector<MotionSnapshot> motion_snapshots) {
    if (index >= (int) targets_.size() || (int) targets_.size() == 0) {
        return 0.0;
    }

    if (index == 0) {
        return targets_[0].value();
    }

    // With linear calculation, TODO use exponents

    double b = motion_snapshots[0].value();
    double d = targets_[0].value() - motion_snapshots[0].value();

    double c = targets_[index].value() - motion_snapshots[index].value();

    double nextTarget = c / d * (ref_ptrs_[0].value() - b);

    if (targets_[index].value() > motion_snapshots[index].value()) {
        if (nextTarget > targets_[index].value()) {
            return targets_[index].value();
        }
    } else if (nextTarget < targets_[index].value()) {
        return targets_[index].value();
    }


    return nextTarget;
 }


 private:
 std::vector<frc846::util::SharePointer>& ref_ptrs_;
 std::vector<MotionTarget>& targets_;
 std::vector<int>& exponents_;

};

}  // namespace frc846::motion

#endif  // FRC846_MOTION_H_