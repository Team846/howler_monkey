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

frc846::motion::MotionTarget setpoints::climb_pivot_{"Pivot", "climb_position", -3_deg};
frc846::motion::MotionTarget setpoints::climb_telescope_{"Telescope", "climb_position", 0_in};
frc846::motion::MotionTarget setpoints::climb_wrist_{"Wrist", "climb_position", 0_in};

frc846::motion::MotionTarget setpoints::pre_climb_pivot_{"Pivot", "pre_climb_position", 100_deg};
frc846::motion::MotionTarget setpoints::pre_climb_telescope_{"Telescope", "pre_climb_position", 0_in};
frc846::motion::MotionTarget setpoints::pre_climb_wrist_{"Wrist", "pre_climb_position", 0_in};

frc846::motion::MotionTarget setpoints::trap_pivot_{"Pivot", "trap_position", 112_deg};
frc846::motion::MotionTarget setpoints::trap_telescope_{"Telescope", "trap_position", 8.5_in};
frc846::motion::MotionTarget setpoints::trap_wrist_{"Wrist", "trap_position", 144.5_deg};

frc846::motion::MotionTarget setpoints::auto_shoot_pivot_{"Pivot", "auto_shoot_position", 39.2_deg};
frc846::motion::MotionTarget setpoints::auto_shoot_telescope_{"Telescope", "auto_shoot_position", 0_in};
frc846::motion::MotionTarget setpoints::auto_shoot_wrist_{"Wrist", "auto_shoot_position", 112_deg};