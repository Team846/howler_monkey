#include "subsystems/setpoints.h"

frc846::Pref<double> setpoints::amp_pivot_{frc846::motion::MotionTarget::preferences_loggable, "Pivot/amp_position", 90};
frc846::Pref<double> setpoints::amp_telescope_{frc846::motion::MotionTarget::preferences_loggable, "Telescope/amp_position", 3};
frc846::Pref<double> setpoints::amp_wrist_{frc846::motion::MotionTarget::preferences_loggable, "Wrist/amp_position", 30};

frc846::Pref<double> setpoints::intake_pivot_{frc846::motion::MotionTarget::preferences_loggable, "Pivot/intake_position", 10};
frc846::Pref<double> setpoints::intake_telescope_{frc846::motion::MotionTarget::preferences_loggable, "Telescope/intake_position", 4.7};
frc846::Pref<double> setpoints::intake_wrist_{frc846::motion::MotionTarget::preferences_loggable, "Wrist/intake_position", 185};

frc846::Pref<double> setpoints::stow_pivot_{frc846::motion::MotionTarget::preferences_loggable, "Pivot/stow_position", 0};
frc846::Pref<double> setpoints::stow_telescope_{frc846::motion::MotionTarget::preferences_loggable, "Telescope/stow_position", 0};
frc846::Pref<double> setpoints::stow_wrist_{frc846::motion::MotionTarget::preferences_loggable, "Wrist/stow_position", 0};

frc846::Pref<double> setpoints::shoot_pivot_{frc846::motion::MotionTarget::preferences_loggable, "Pivot/shoot_position", 60};
frc846::Pref<double> setpoints::shoot_telescope_{frc846::motion::MotionTarget::preferences_loggable, "Telescope/shoot_position", 0};
frc846::Pref<double> setpoints::point_blank_wrist_{frc846::motion::MotionTarget::preferences_loggable, "Wrist/shoot_point_blank_", 43.55};

frc846::Pref<double> setpoints::climb_pivot_{frc846::motion::MotionTarget::preferences_loggable, "Pivot/climb_position", -3};
frc846::Pref<double> setpoints::climb_telescope_{frc846::motion::MotionTarget::preferences_loggable, "Telescope/climb_position", 0};
frc846::Pref<double> setpoints::climb_wrist_{frc846::motion::MotionTarget::preferences_loggable, "Wrist/climb_position", 0};

frc846::Pref<double> setpoints::pre_climb_pivot_{frc846::motion::MotionTarget::preferences_loggable, "Pivot/pre_climb_position", 100};
frc846::Pref<double> setpoints::pre_climb_telescope_{frc846::motion::MotionTarget::preferences_loggable, "Telescope/pre_climb_position", 0};
frc846::Pref<double> setpoints::pre_climb_wrist_{frc846::motion::MotionTarget::preferences_loggable, "Wrist/pre_climb_position", 0};

frc846::Pref<double> setpoints::trap_pivot_{frc846::motion::MotionTarget::preferences_loggable, "Pivot/trap_position", 112};
frc846::Pref<double> setpoints::trap_telescope_{frc846::motion::MotionTarget::preferences_loggable, "Telescope/trap_position", 8.5};
frc846::Pref<double> setpoints::trap_wrist_{frc846::motion::MotionTarget::preferences_loggable, "Wrist/trap_position", 144.5};

frc846::Pref<double> setpoints::auto_shoot_pivot_{frc846::motion::MotionTarget::preferences_loggable, "Pivot/auto_shoot_position", 39.2};
frc846::Pref<double> setpoints::auto_shoot_telescope_{frc846::motion::MotionTarget::preferences_loggable, "Telescope/auto_shoot_position", 0};
frc846::Pref<double> setpoints::auto_shoot_wrist_{frc846::motion::MotionTarget::preferences_loggable, "Wrist/auto_shoot_position", 112};