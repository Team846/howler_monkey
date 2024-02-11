#include "commands/teleop_positioning_command.h"

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

  bool running_prep_speaker = driver_.readings().right_trigger;

  PivotTarget pivot_target = pivot_.GetTarget();
  TelescopeTarget telescope_target = telescope_.GetTarget();
  WristTarget wrist_target = wrist_.GetTarget();   

  if (pivot_up_manual || pivot_down_manual) {
    if (pivot_up_manual) {
      pivot_target.pivot_output = 0.1;
    } else if (pivot_down_manual) {
      pivot_target.pivot_output = -0.1;
    }

    pivotHasRun = true;
  } else if (running_prep_speaker) {
    double SPEAKER_HEIGHT = 3.7;
    double LAUNCH_VELOCITY = 27.0;
    double GRAVITY = 32.0;
    double shooting_dist = (field::points::kSpeaker() - drivetrain_.readings().pose.point).Magnitude().to<double>();

    std::cout << "SD: " << shooting_dist << std::endl;

    // t = vsin(theta)/g
    // dist = vcos(theta)t
    // dist = v^2sin(theta)cos(theta)/g
    // dist*g/(v^2) = sin(theta)cos(theta)
    // dist*g/(v^2) = 1/2 sin(2theta)
    // theta = arcsin(2*dist*g/(v^2))/2

    // OR

    // h = 1/2 * g * t^2
    // t = sqrt(2h/g)
    // dist = vcos(theta)*t
    // dist = vcos(theta)*sqrt(2h/g)
    // theta = arccos(dist/(v*sqrt(2h/g)))

    units::degree_t theta = units::radian_t(std::asin(2*shooting_dist
      *GRAVITY/(LAUNCH_VELOCITY * LAUNCH_VELOCITY))/2);
    units::degree_t theta2 = units::radian_t(std::acos(shooting_dist/(LAUNCH_VELOCITY
      *std::sqrt(2*SPEAKER_HEIGHT / GRAVITY)))); //theta 2 seems to work?

    //pivot_target.pivot_output = theta;

    std::cout << "ANGLE: " << theta.to<double>() << std::endl;
    std::cout << "ANGLE 2: " << theta2.to<double>() << std::endl;
    std::cout << "RAW: " << 2*shooting_dist*GRAVITY/(LAUNCH_VELOCITY * LAUNCH_VELOCITY) << std::endl;

    pivotHasRun = true;
  } else if (pivotHasRun) {
    pivot_target.pivot_output = pivot_.readings().pivot_position;
    pivotHasRun = false;
  }

  if (telescope_in_manual || telescope_out_manual) {
    if (telescope_in_manual) {
      telescope_target.extension = -0.1;
    } else if (telescope_out_manual) {
      telescope_target.extension = 0.1;
    }
    
    telescopeHasRun = true;
  } else if (running_prep_speaker) {
    telescopeHasRun = true;
  } else if (telescopeHasRun) {
    telescope_target.extension = telescope_.readings().extension;
    telescopeHasRun = false;
  }

  if (wrist_up_manual || wrist_down_manual) {
    if (wrist_up_manual) {
      wrist_target.wrist_output = 0.1;
    } else if (wrist_down_manual) {
      wrist_target.wrist_output = -0.1;
    }

    wristHasRun = true;
  } else if (running_prep_speaker) {

    wristHasRun = true;
  } else if (wristHasRun) {
    wrist_target.wrist_output = wrist_.readings().wrist_position;
    wristHasRun = false;
  }

  pivot_.SetTarget(pivot_target);
  telescope_.SetTarget(telescope_target);
  wrist_.SetTarget(wrist_target);
  
}

bool TeleopPositioningCommand::IsFinished() { return false; }