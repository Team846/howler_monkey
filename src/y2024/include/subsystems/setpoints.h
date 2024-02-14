#ifndef y2024_SETPOINTS_H_
#define y2024_SETPOINTS_H_

#include "frc846/control/motion.h"

struct setpoints {
  static frc846::motion::MotionProfile intake_profile() { 
    return frc846::motion::MotionProfile{
        {
            frc846::util::SharePointer("pivot_position"),
            frc846::util::SharePointer("telescope_extension"),
            frc846::util::SharePointer("wrist_position")
        }, 
        {
            frc846::motion::MotionTarget("Pivot", "intake_position", 0_deg),
            frc846::motion::MotionTarget("Telescope", "intake_position", 0_deg),
            frc846::motion::MotionTarget("Wrist", "intake_position", 0_deg),
        }, 
        {
            1, 1, 1
        }
    };
  }

  static frc846::motion::MotionProfile stow_profile() { 
    return frc846::motion::MotionProfile{
        {
            frc846::util::SharePointer("pivot_position"),
            frc846::util::SharePointer("telescope_extension"),
            frc846::util::SharePointer("wrist_position")
        }, 
        {
            frc846::motion::MotionTarget("Pivot", "stow_position", 0_deg),
            frc846::motion::MotionTarget("Telescope", "stow_position", 0_deg),
            frc846::motion::MotionTarget("Wrist", "stow_position", 0_deg),
        }, 
        {
            1, 1, 1
        }
    };
  }
};

#endif  // y2024_SETPOINTS_H_