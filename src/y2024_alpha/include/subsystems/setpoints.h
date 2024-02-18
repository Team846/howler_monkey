#ifndef y2024_SETPOINTS_H_
#define y2024_SETPOINTS_H_

#include "frc846/control/motion.h"

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
};

#endif  // y2024_SETPOINTS_H_