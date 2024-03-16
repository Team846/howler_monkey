#ifndef y2024_SETPOINTS_H_
#define y2024_SETPOINTS_H_

#include "frc846/control/motion.h"

struct CoordinateTarget {
  frc846::Pref<double> shooting_angle;
  frc846::Pref<double> forward_axis;
  frc846::Pref<double> upward_axis;
};

struct setpoints {
  static frc846::Pref<double> amp_pivot_;
  static frc846::Pref<double> amp_telescope_;
  static frc846::Pref<double> amp_wrist_;

  static double kAmp(int kIndex) { 
    switch (kIndex) {
        case 2:
            return amp_wrist_.value();
        case 1:
            return amp_telescope_.value();
        default:
            return amp_pivot_.value();
    }
  }

  static frc846::Pref<double> intake_pivot_;
  static frc846::Pref<double> intake_telescope_;
  static frc846::Pref<double> intake_wrist_;

  static double kIntake(int kIndex) { 
    switch (kIndex) {
        case 2:
            return intake_wrist_.value();
        case 1:
            return intake_telescope_.value();
        default:
            return intake_pivot_.value();
    }
  }

  static frc846::Pref<double> source_pivot_;
  static frc846::Pref<double> source_telescope_;
  static frc846::Pref<double> source_wrist_;

  static double kSource(int kIndex) { 
    switch (kIndex) {
        case 2:
            return source_wrist_.value();
        case 1:
            return source_telescope_.value();
        default:
            return source_pivot_.value();
    }
  }

  static frc846::Pref<double> stow_pivot_;
  static frc846::Pref<double> stow_telescope_;
  static frc846::Pref<double> stow_wrist_;

  static double kStow(int kIndex) { 
    switch (kIndex) {
        case 2:
            return stow_wrist_.value();
        case 1:
            return stow_telescope_.value();
        default:
            return stow_pivot_.value();
    }
  }

  static frc846::Pref<double> shoot_pivot_;
  static frc846::Pref<double> shoot_telescope_;
  static frc846::Pref<double> point_blank_wrist_;

  static double kShoot(int kIndex) { 
    switch (kIndex) {
        case 2:
            return point_blank_wrist_.value();
        case 1:
            return shoot_telescope_.value();
        default:
            return shoot_pivot_.value();
    }
  }

  static frc846::Pref<double> auto_shoot_pivot_;
  static frc846::Pref<double> auto_shoot_telescope_;
  static frc846::Pref<double> auto_shoot_wrist_;

  static double kAutoShoot(int kIndex) { 
    switch (kIndex) {
        case 2:
            return auto_shoot_wrist_.value();
        case 1:
            return auto_shoot_telescope_.value();
        default:
            return auto_shoot_pivot_.value();
    }
  }

  static frc846::Pref<double> climb_pivot_;
  static frc846::Pref<double> climb_telescope_;
  static frc846::Pref<double> climb_wrist_;

  static double kClimb(int kIndex) { 
    switch (kIndex) {
        case 2:
            return climb_wrist_.value();
        case 1:
            return climb_telescope_.value();
        default:
            return climb_pivot_.value();
    }
  }

  static frc846::Pref<double> pre_climb_pivot_;
  static frc846::Pref<double> pre_climb_telescope_;
  static frc846::Pref<double> pre_climb_wrist_;

  static double kPreClimb(int kIndex) { 
    switch (kIndex) {
        case 2:
            return pre_climb_wrist_.value();
        case 1:
            return pre_climb_telescope_.value();
        default:
            return pre_climb_pivot_.value();
    }
  }

  static frc846::Pref<double> trap_pivot_;
  static frc846::Pref<double> trap_telescope_;
  static frc846::Pref<double> trap_wrist_;

  static double kTrap(int kIndex) { 
    switch (kIndex) {
        case 2:
            return trap_wrist_.value();
        case 1:
            return trap_telescope_.value();
        default:
            return trap_pivot_.value();
    }
  }
};

#endif  // y2024_SETPOINTS_H_