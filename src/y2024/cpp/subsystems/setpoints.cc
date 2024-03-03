#include "subsystems/setpoints.h"

frc846::motion::MotionTarget setpoints::amp_pivot_{"Pivot", "amp_position", 90_deg};
frc846::motion::MotionTarget setpoints::amp_telescope_{"Telescope", "amp_position", 3_in};
frc846::motion::MotionTarget setpoints::amp_wrist_{"Wrist", "amp_position", 30_deg};

frc846::motion::MotionTarget setpoints::intake_pivot_{"Pivot", "intake_position", 10_deg};
frc846::motion::MotionTarget setpoints::intake_telescope_{"Telescope", "intake_position", 4.7_in};
frc846::motion::MotionTarget setpoints::intake_wrist_{"Wrist", "intake_position", 185_deg};

frc846::motion::MotionTarget setpoints::stow_pivot_{"Pivot", "stow_position", 0_deg};
frc846::motion::MotionTarget setpoints::stow_telescope_{"Telescope", "stow_position", 0_in};
frc846::motion::MotionTarget setpoints::stow_wrist_{"Wrist", "stow_position", 0_deg};

frc846::motion::MotionTarget setpoints::shoot_pivot_{"Pivot", "shoot_position", 60_deg};
frc846::motion::MotionTarget setpoints::shoot_telescope_{"Telescope", "shoot_position", 0_in};
frc846::motion::MotionTarget setpoints::point_blank_wrist_{"Wrist", "shoot_point_blank_", 43.55_deg};

frc846::motion::MotionTarget setpoints::trap_pivot_{"Pivot", "trap_position", 112_deg};
frc846::motion::MotionTarget setpoints::trap_telescope_{"Telescope", "trap_position", 8.5_in};
frc846::motion::MotionTarget setpoints::trap_wrist_{"Wrist", "trap_position", 144.5_deg};