#include "commands/teleop_positioning_command.h"
#include "subsystems/setpoints.h"

#include <utility>

#include "frc846/util/math.h"
#include "subsystems/field.h"
#include "subsystems/swerve_module.h"
#include "frc846/loggable.h"

TeleopPositioningCommand::TeleopPositioningCommand(RobotContainer& container)
    : driver_(container.driver_),
      drivetrain_(container.drivetrain_),
      pivot_(container.pivot_),
      telescope_(container.telescope_),
      wrist_(container.wrist_) {
  AddRequirements({&pivot_, &telescope_, &wrist_});
  SetName("teleop_positioning_command");
  firstPositionRound = true;
}

void TeleopPositioningCommand::Execute() {
  DrivetrainTarget drivetrain_target;

  // -----BUTTON MAPPINGS-----

  // Left bumper   | robot centric translation
  // Right bumper  | precision drive


  bool telescope_in_manual = driver_.readings().a_button;
  bool telescope_out_manual = driver_.readings().b_button;
  bool pivot_down_manual = driver_.readings().x_button;
  bool pivot_up_manual = driver_.readings().y_button;
  bool wrist_up_manual = driver_.readings().pov == frc846::XboxPOV::kUp;
  bool wrist_down_manual = driver_.readings().pov == frc846::XboxPOV::kDown;

  bool running_intake_position = driver_.readings().left_trigger;
  bool running_amp_position = driver_.readings().left_bumper;
  bool running_prep_speaker = driver_.readings().right_trigger;

  PivotTarget pivot_target = pivot_.GetTarget();
  TelescopeTarget telescope_target = telescope_.GetTarget();
  WristTarget wrist_target = wrist_.GetTarget();   

  if (!running_intake_position) {
    firstPositionRound = true;
  }

  if (pivot_up_manual || pivot_down_manual) {
    firstPositionRound = true;

    if (pivot_up_manual) {
      pivot_target.pivot_output = 0.1;
    } else if (pivot_down_manual) {
      pivot_target.pivot_output = -0.1;
    }

    pivotHasRun = false;
  } else if (running_prep_speaker) {
    firstPositionRound = true;

    pivot_target.pivot_output = 60_deg;

    pivotHasRun = true;
  } else if (running_intake_position) {
    if (firstPositionRound) {
      double nextPivotTarget = setpoints::kIntake(0).value();
      pivot_target.pivot_output = units::degree_t(nextPivotTarget);
    }

    firstPositionRound = false;
    pivotHasRun = true;
  } else if (running_amp_position) {
    if (firstPositionRound) {
      double nextPivotTarget = setpoints::kAmp(0).value();
      pivot_target.pivot_output = units::degree_t(nextPivotTarget);
    }

    firstPositionRound = false;
    pivotHasRun = true;
  } else if (pivotHasRun) {
    firstPositionRound = true;
    double nextPivotTarget = setpoints::kStow(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);
  } else {
    firstPositionRound = true;
    pivot_target.pivot_output = 0.0;
  }

  if (telescope_in_manual || telescope_out_manual) {
    if (telescope_in_manual) {
      telescope_target.extension = -0.1;
    } else if (telescope_out_manual) {
      telescope_target.extension = 0.1;
    }
    
    telescopeHasRun = false;
  } else if (running_prep_speaker) {
    telescope_target.extension = 0_in;

    telescopeHasRun = true;
  } else if (running_intake_position) {
    double nextTelescopeTarget = setpoints::kIntake(1).value();
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    telescopeHasRun = true;
  } else if (running_amp_position) {
    double nextTelescopeTarget = setpoints::kAmp(1).value();
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    telescopeHasRun = true;
  } else if (telescopeHasRun) {
    double nextTelescopeTarget = setpoints::kStow(1).value();
    telescope_target.extension = units::inch_t(nextTelescopeTarget);
  } else {
    telescope_target.extension = 0.0;
  }

  if (wrist_up_manual || wrist_down_manual) {
    if (wrist_up_manual) {
      wrist_target.wrist_output = 0.1;
    } else if (wrist_down_manual) {
      wrist_target.wrist_output = -0.1;
    }
    wristHasRun = false;
  } else if (running_prep_speaker) {
    double SPEAKER_HEIGHT = 3.7;
    double LAUNCH_VELOCITY = 29.0;
    double GRAVITY = 32.0;
    double shooting_dist = (field::points::kSpeakerTeleop() - drivetrain_.readings().pose.point).Magnitude().to<double>();


    // h = 1/2 * g * t^2
    // t = sqrt(2h/g)
    // dist = vcos(theta)*t
    // dist = vcos(theta)*sqrt(2h/g)
    // theta = arccos(dist/(v*sqrt(2h/g)))

    // While moving:

    // h = 1/2 * g * t^2
    // t = sqrt(2h/g)
    // dist = (vcos(theta) + r_v_comp)*t
    // dist = (vcos(theta) + r_v_comp)*sqrt(2h/g)
    // vcos(theta) = dist/sqrt(2h/g) - r_v_comp
    // theta = arccos((dist/sqrt(2h/g) - r_v_comp)/v)


    auto robot_velocity = drivetrain_.readings().velocity;
    auto point_target = (field::points::kSpeakerTeleop() - drivetrain_.readings().pose.point);

    double robot_velocity_in_component = 
      (robot_velocity.x.to<double>() * point_target.x.to<double>() + 
        robot_velocity.y.to<double>() * point_target.y.to<double>())/point_target.Magnitude().to<double>();

    units::degree_t theta_adjusted = units::radian_t(std::abs(std::acos((shooting_dist/
      (std::sqrt(2*SPEAKER_HEIGHT / GRAVITY)) - robot_velocity_in_component)/LAUNCH_VELOCITY)));

    if (theta_adjusted < 7_deg) {
      DriverTarget driver_target{true};
      driver_.SetTarget(driver_target);
    } else {
      DriverTarget driver_target{false};
      driver_.SetTarget(driver_target);
    }

    wrist_target.wrist_output = (theta_adjusted); //add zero_offset, replace 45_deg with pivot

    wristHasRun = true;
  } else if (running_intake_position) {
    double nextWristTarget = setpoints::kIntake(2).value();
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    wristHasRun = true;
  } else if (running_amp_position) {
    double nextWristTarget = setpoints::kAmp(2).value();
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    wristHasRun = true;
  } else if (wristHasRun) {
    double nextWristTarget = setpoints::kStow(2).value();
    wrist_target.wrist_output = units::degree_t(nextWristTarget);
  } else {
    wrist_target.wrist_output = 0.0;
  }

  pivot_.SetTarget(pivot_target);
  telescope_.SetTarget(telescope_target);
  wrist_.SetTarget(wrist_target);
}

bool TeleopPositioningCommand::IsFinished() { return false; }