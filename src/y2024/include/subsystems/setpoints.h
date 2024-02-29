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

  static frc846::motion::MotionTarget shoot_x_;
  static frc846::motion::MotionTarget shoot_y_;
  static frc846::motion::MotionTarget point_blank_shooting_angle_;

  
  static CoordinateTarget kShootCoordinates(int kIndex) { 
    return {point_blank_shooting_angle_, shoot_x_, shoot_y_};
  }
};

#endif  // y2024_SETPOINTS_H_