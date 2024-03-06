#ifndef y2024_SETPOINTS_H_
#define y2024_SETPOINTS_H_

#include "frc846/control/motion.h"

struct CoordinateTarget {
  frc846::motion::MotionTarget shooting_angle;
  frc846::motion::MotionTarget forward_axis;
  frc846::motion::MotionTarget upward_axis;
};

struct setpoints {
  static frc846::motion::MotionTarget amp_pivot_;
  static frc846::motion::MotionTarget amp_telescope_;
  static frc846::motion::MotionTarget amp_wrist_;

  static frc846::motion::MotionTarget kAmp(int kIndex) { 
    switch (kIndex) {
        case 2:
            return amp_wrist_;
        case 1:
            return amp_telescope_;
        default:
            return amp_pivot_;
    }
  }

  static frc846::motion::MotionTarget intake_pivot_;
  static frc846::motion::MotionTarget intake_telescope_;
  static frc846::motion::MotionTarget intake_wrist_;

  static frc846::motion::MotionTarget kIntake(int kIndex) { 
    switch (kIndex) {
        case 2:
            return intake_wrist_;
        case 1:
            return intake_telescope_;
        default:
            return intake_pivot_;
    }
  }

  static frc846::motion::MotionTarget stow_pivot_;
  static frc846::motion::MotionTarget stow_telescope_;
  static frc846::motion::MotionTarget stow_wrist_;

  static frc846::motion::MotionTarget kStow(int kIndex) { 
    switch (kIndex) {
        case 2:
            return stow_wrist_;
        case 1:
            return stow_telescope_;
        default:
            return stow_pivot_;
    }
  }

  static frc846::motion::MotionTarget shoot_pivot_;
  static frc846::motion::MotionTarget shoot_telescope_;
  static frc846::motion::MotionTarget point_blank_wrist_;

  static frc846::motion::MotionTarget kShoot(int kIndex) { 
    switch (kIndex) {
        case 2:
            return point_blank_wrist_;
        case 1:
            return shoot_telescope_;
        default:
            return shoot_pivot_;
    }
  }

  static frc846::motion::MotionTarget auto_shoot_pivot_;
  static frc846::motion::MotionTarget auto_shoot_telescope_;
  static frc846::motion::MotionTarget auto_shoot_wrist_;

  static frc846::motion::MotionTarget kAutoShoot(int kIndex) { 
    switch (kIndex) {
        case 2:
            return auto_shoot_wrist_;
        case 1:
            return auto_shoot_telescope_;
        default:
            return auto_shoot_pivot_;
    }
  }

  static frc846::motion::MotionTarget climb_pivot_;
  static frc846::motion::MotionTarget climb_telescope_;
  static frc846::motion::MotionTarget climb_wrist_;

  static frc846::motion::MotionTarget kClimb(int kIndex) { 
    switch (kIndex) {
        case 2:
            return climb_wrist_;
        case 1:
            return climb_telescope_;
        default:
            return climb_pivot_;
    }
  }

  static frc846::motion::MotionTarget pre_climb_pivot_;
  static frc846::motion::MotionTarget pre_climb_telescope_;
  static frc846::motion::MotionTarget pre_climb_wrist_;

  static frc846::motion::MotionTarget kPreClimb(int kIndex) { 
    switch (kIndex) {
        case 2:
            return pre_climb_wrist_;
        case 1:
            return pre_climb_telescope_;
        default:
            return pre_climb_pivot_;
    }
  }

  static frc846::motion::MotionTarget trap_pivot_;
  static frc846::motion::MotionTarget trap_telescope_;
  static frc846::motion::MotionTarget trap_wrist_;

  static frc846::motion::MotionTarget kTrap(int kIndex) { 
    switch (kIndex) {
        case 2:
            return trap_wrist_;
        case 1:
            return trap_telescope_;
        default:
            return trap_pivot_;
    }
  }
};

#endif  // y2024_SETPOINTS_H_