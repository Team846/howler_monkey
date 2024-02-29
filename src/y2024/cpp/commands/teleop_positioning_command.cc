#include "commands/teleop_positioning_command.h"
#include "subsystems/setpoints.h"

#include <utility>

#include "frc846/util/math.h"
#include "subsystems/field.h"
#include "subsystems/swerve_module.h"
#include "frc846/loggable.h"

class TeleopShootingCalculator {
  private:
    static double pow(double base, int exponent) {
      for (int i = 0; i < exponent; i++) {
        base *= base;
      }
      return base;
    }

    static constexpr double pi = 3.14159265;

    static double radians(double degs) {
      return degs * pi / 180;
    }

    static double degs(double radians) {
      return radians * 180 / pi;
    }

    static constexpr double g = 32.0;

    static constexpr double h = 78/12 - 26/12;
    static constexpr double l = 14.0 / 12.0;

    static constexpr double v = 25.5;

    static constexpr double k = 0; //0.43;
    static constexpr double w = 0; //105.0 * pi / 180.0;

  public:

  static double f_of_x(double x, double d, double r_v, double r_o) {
    double cosx = cos(x);
    double sinx = sin(x);
    return ((v*sinx)*(d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v)
                -1.0/2.0*g*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))  
                    +l*sinx/2+k*sin(x+w)-h);
  }

  static double f_prime(double x, double d, double r_v, double r_o) {
    double cosx = cos(x);
    double sinx = sin(x);

    double t = 0.0;
    double t_prime = 0.0;

    try {
      t = (d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/v*cosx))*(r_o/v*cosx)+r_v);
      t_prime = (l*r_v*v*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v)) + l*r_o*r_o*cosx*cosx*cosx - 4*d*r_o*r_o*cosx*cosx + 2*d*v*v)*sinx /
                      (2*v*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v))*(r_v+v*cosx*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v))));
    } catch (std::exception exc) {
      std::cout << "Out of Range" << std::endl;
    }

    return v*cosx*t + v*sinx*t_prime + l*cosx/2 - 1.0/2.0*g*2*t*t_prime;
  }

  static double calculate(double d, double r_v, double r_o, 
    double initial_guess = radians(30), double tolerance=0.04, double max_iterations=200) {
    double x = initial_guess;
    for (int i = 0; i < max_iterations; i++) {
        auto fx = f_of_x(x, d, r_v, r_o);
        if (std::abs(fx) < tolerance) {
          return degs(x);
        }

        x -= std::min(radians(120.0/std::max(14, (i+1))), 
          std::max(radians(-120.0/std::max(14, (i+1))), fx / f_prime(x, d, r_v, r_o))) / 10.0;

        if (x < 0.0) x = radians(30);
        else if (x > pi / 2) x = radians(30);
    }

    return 0.0;
  }
};

struct RawPositions {
  units::degree_t pivot_angle;
  units::degree_t wrist_angle;
  units::inch_t extension;
};

struct CoordinatePositions {
  units::degree_t shooting_angle;
  units::inch_t forward_axis;
  units::inch_t upward_axis;
};

class InverseKinematics {
  private:
  static constexpr double pivotToWrist = 20.05;
  static constexpr double pivotToWristOffset = -4.25;
  static constexpr double wristToFlywheels = 6.2;

  static constexpr double pivotToGround = 17.5;

  static constexpr double pivotToCenter = 9.5; //FIX

  static constexpr double robotWidth = 28;

  static double pow(double base, int exponent) {
    for (int i = 0; i < exponent; i++) {
      base *= base;
    }
    return base;
  }
  
  //6.2, 17.5, (20.5, 4.25)
  public:
  static RawPositions toRaw(CoordinatePositions pos) {
    RawPositions raw{};

    double pivotForwardComponent = (pos.forward_axis - units::inch_t(wristToFlywheels)*units::math::cos(pos.shooting_angle)).to<double>() 
      - pivotToCenter;
    double pivotUpwardComponent = (pos.upward_axis - units::inch_t(wristToFlywheels)*units::math::sin(pos.shooting_angle)).to<double>();
    
    raw.extension = units::inch_t(std::sqrt(pow(pivotForwardComponent, 2) + pow(pivotUpwardComponent, 2) - pow(pivotToWristOffset, 2)) - pivotToWrist);

    double pivotDistanceHypotenuse = (std::sqrt(pow(pivotToWristOffset, 2) 
      + pow(pivotToWrist + raw.extension.to<double>(), 2)));

      
    // raw.wrist_angle = units::degree_t(pos.shooting_angle.to<double>() - std::acos((pos.forward_axis.to<double>() - 
    //   wristToFlywheels * std::cos(pos.shooting_angle.to<double>()))/pivotDistanceHypotenuse));

    units::degree_t pivotTotalAngle = units::degree_t(std::atan2(pivotUpwardComponent, pivotForwardComponent));

    units::degree_t pivotAddedAngle = units::degree_t(std::atan2(pivotToWristOffset, pivotToWrist + raw.extension.to<double>()));

    raw.pivot_angle = pivotTotalAngle - pivotAddedAngle - 17.0_deg;

    raw.wrist_angle = pos.shooting_angle - pivotTotalAngle - 47.0_deg;

    return raw;
  }

  static CoordinatePositions toCoordinate(RawPositions pos) {
    CoordinatePositions coordinate{};

    pos.pivot_angle += 17_deg;
    pos.wrist_angle += 49_deg;
  
    double truePivotAngle = (std::atan2(pivotToWristOffset,
          pos.extension.to<double>()+pivotToWrist) + (pos.pivot_angle.to<double>()));

    double pivotDistanceHypotenuse = (std::sqrt(pow(pivotToWristOffset, 2) 
      + pow(pivotToWrist + pos.extension.to<double>(), 2)));

    coordinate.upward_axis = units::inch_t(pivotToGround + 
       pivotDistanceHypotenuse * std::sin(truePivotAngle) + 
        wristToFlywheels * std::sin(truePivotAngle - pos.wrist_angle.to<double>()));

    coordinate.forward_axis = units::inch_t(pivotDistanceHypotenuse * std::cos(truePivotAngle) + 
      wristToFlywheels * std::cos(truePivotAngle - pos.wrist_angle.to<double>()));

    coordinate.shooting_angle = units::degree_t(truePivotAngle) - pos.wrist_angle; 

    return coordinate;
  }
};

TeleopPositioningCommand::TeleopPositioningCommand(RobotContainer& container)
    : driver_(container.driver_),
      operator_(container.operator_),
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



  bool telescope_in_manual = operator_.readings().a_button;
  bool telescope_out_manual = operator_.readings().b_button;
  bool pivot_down_manual = operator_.readings().x_button;
  bool pivot_up_manual = operator_.readings().y_button;
  bool wrist_up_manual = operator_.readings().pov == frc846::XboxPOV::kUp;
  bool wrist_down_manual = operator_.readings().pov == frc846::XboxPOV::kDown;

  bool running_intake_position = driver_.readings().left_trigger;
  bool running_amp_position = driver_.readings().left_bumper;
  bool running_prep_speaker = driver_.readings().right_trigger;

  bool temporary_pre_climb = operator_.readings().left_trigger;
  bool temporary_climb = operator_.readings().right_trigger;

  PivotTarget pivot_target = pivot_.GetTarget();
  TelescopeTarget telescope_target = telescope_.GetTarget();
  WristTarget wrist_target = wrist_.GetTarget();   

  if (pivot_up_manual || pivot_down_manual) {
    if (pivot_up_manual) {
      mpiv_adj += 40.0 / 50.0;
    } else if (pivot_down_manual) {
      mpiv_adj -= 40.0 / 50.0;
    }
    pivotHasRun = false;
  }

  if (telescope_in_manual || telescope_out_manual) {
    if (telescope_in_manual) {
      mtele_adj += 11.0 / (2.0 * 50.0);
    } else if (telescope_out_manual) {
      mtele_adj -= 11.0 / (2.0 * 50.0);
    }
    telescopeHasRun = false;
  }

  if (wrist_up_manual || wrist_down_manual) {
    if (wrist_up_manual) {
      mwr_adj += 40.0 / 50.0;
    } else if (wrist_down_manual) {
      mwr_adj -= 40.0 / 50.0;
    }
    wristHasRun = false; // remove? fix?
  }

  double translate_x = frc846::util::HorizontalDeadband(
      operator_.readings().left_stick_x, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
  double translate_u = frc846::util::HorizontalDeadband(
      operator_.readings().left_stick_y, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);

  if (std::abs(translate_x) > 0.02) {
    mx_adj += 6.0 / 50.0 * translate_x;
  }

  if (std::abs(translate_u) > 0.02) {
    mu_adj += 6.0 / 50.0 * translate_u;
  }

  if (wrist_up_manual || wrist_down_manual) {
    if (wrist_up_manual) {
      ms_adj += 40.0 / 50.0;
    } else if (wrist_down_manual) {
      ms_adj -= 40.0 / 50.0;
    }
  }

  if (temporary_climb) {
    pivot_target.pivot_output = 3_deg + units::degree_t(mpiv_adj);

    pivotHasRun = true;
  } else if (temporary_pre_climb) {
    pivot_target.pivot_output = 107_deg + units::degree_t(mpiv_adj);

    pivotHasRun = true;
  } else if (running_prep_speaker) {
    double nextPivotTarget = setpoints::kShoot(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget + mpiv_adj);

    std::cout << InverseKinematics::toCoordinate({units::degree_t(setpoints::kIntake(0).value()), 
      units::degree_t(setpoints::kIntake(2).value()),
         units::inch_t(setpoints::kIntake(1).value())}).forward_axis.to<double>() << std::endl;
    std::cout << InverseKinematics::toCoordinate({units::degree_t(setpoints::kIntake(0).value()), 
      units::degree_t(setpoints::kIntake(2).value()),
         units::inch_t(setpoints::kIntake(1).value())}).upward_axis.to<double>() << std::endl;
    std::cout << InverseKinematics::toCoordinate({units::degree_t(setpoints::kIntake(0).value()), 
      units::degree_t(setpoints::kIntake(2).value()),
         units::inch_t(setpoints::kIntake(1).value())}).shooting_angle.to<double>() << std::endl;
    std::cout << "endback" << std::endl;
    std::cout << InverseKinematics::toRaw(InverseKinematics::toCoordinate({units::degree_t(setpoints::kIntake(0).value()), 
    units::degree_t(setpoints::kIntake(2).value()),
        units::inch_t(setpoints::kIntake(1).value())})).pivot_angle.to<double>() << std::endl;

    pivotHasRun = true;
  } else if (running_intake_position) {
    double nextPivotTarget = setpoints::kIntake(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget + mpiv_adj);

    pivotHasRun = true;
  } else if (running_amp_position) {
    double nextPivotTarget = setpoints::kAmp(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget + mpiv_adj);

    pivotHasRun = true;
  } else if (pivotHasRun) {
    double nextPivotTarget = setpoints::kStow(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    mpiv_adj = 0.0;
  } if (pivot_up_manual) {
    pivot_target.pivot_output = 0.13;
    lastPivotManual = true;
  } else if (pivot_down_manual) {
    pivot_target.pivot_output = -0.13;
    lastPivotManual = true;
  } else if (lastPivotManual) {
    pivot_target.pivot_output = pivot_.readings().pivot_position;
    lastPivotManual = false;
  }

  if (temporary_climb || temporary_pre_climb) {
    telescope_target.extension = units::inch_t(2);

    telescopeHasRun = true;
  } if (running_prep_speaker) {
    double nextTelescopeTarget = setpoints::kShoot(1).value();
    telescope_target.extension = units::inch_t(nextTelescopeTarget + mtele_adj);

    telescopeHasRun = true;
  } else if (running_intake_position) {
    double nextTelescopeTarget = setpoints::kIntake(1).value();
    telescope_target.extension = units::inch_t(nextTelescopeTarget + mtele_adj);

    telescopeHasRun = true;
  } else if (running_amp_position) {
    double nextTelescopeTarget = setpoints::kAmp(1).value();
    telescope_target.extension = units::inch_t(nextTelescopeTarget + mtele_adj);

    telescopeHasRun = true;
  } else if (telescopeHasRun) {
    double nextTelescopeTarget = setpoints::kStow(1).value();
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    mtele_adj = 0.0;
  } if (telescope_out_manual) {
    telescope_target.extension = 0.1;
    lastTeleManual = true;
  } else if (telescope_in_manual) {
    telescope_target.extension = -0.1;
    lastTeleManual = true;
  } else if (lastTeleManual) {
    telescope_target.extension = telescope_.readings().extension;
    lastTeleManual = false;
  }

  if (temporary_climb || temporary_pre_climb) {
    double nextWristTarget = setpoints::kStow(2).value();
    wrist_target.wrist_output = units::degree_t(nextWristTarget + mwr_adj);

    wristHasRun = true;
  } else if (running_prep_speaker) {
    double shooting_dist = (field::points::kSpeakerTeleop() - drivetrain_.readings().pose.point).Magnitude().to<double>();

    auto robot_velocity = drivetrain_.readings().velocity;
    auto point_target = (field::points::kSpeakerTeleop() - drivetrain_.readings().pose.point);

    double robot_velocity_in_component = 
      (robot_velocity.x.to<double>() * point_target.x.to<double>() + 
        robot_velocity.y.to<double>() * point_target.y.to<double>())/point_target.Magnitude().to<double>();


    double robot_velocity_orth_component = std::sqrt(robot_velocity.Magnitude().to<double>()*
      robot_velocity.Magnitude().to<double>() - robot_velocity_in_component * robot_velocity_in_component);


    shooting_dist = (40.0 + 26.0) / 12.0;

    units::degree_t theta = units::degree_t(TeleopShootingCalculator::calculate(shooting_dist, 
      0.0, 0.0));

    if (theta < 1_deg) {
      DriverTarget driver_target{true};
      driver_.SetTarget(driver_target);
    } else {
      DriverTarget driver_target{false};
      driver_.SetTarget(driver_target);
    }

    std::cout << theta.to<double>() << std::endl;

    wrist_target.wrist_output = 90_deg + units::degree_t(frc846::util::ShareTables::GetDouble("pivot_position")) - 17.5_deg
      - wrist_.wrist_home_offset_.value() + units::degree_t(theta) + units::degree_t(mwr_adj); //units::degree_t(setpoints::point_blank_wrist_.value());

    wristHasRun = true;
  } else if (running_intake_position) {
    double nextWristTarget = setpoints::kIntake(2).value();
    wrist_target.wrist_output = units::degree_t(nextWristTarget + mwr_adj);

    wristHasRun = true;
  } else if (running_amp_position) {
    double nextWristTarget = setpoints::kAmp(2).value();
    wrist_target.wrist_output = units::degree_t(nextWristTarget + mwr_adj);

    wristHasRun = true;
  } else if (wristHasRun) {
    double nextWristTarget = setpoints::kStow(2).value();
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    mwr_adj = 0.0;
  } if (wrist_up_manual) {
    wrist_target.wrist_output = 0.15;
    lastWristManual = true;
  } else if (wrist_down_manual) {
    wrist_target.wrist_output = -0.15;
    lastWristManual = true;
  } else if (lastWristManual) {
    wrist_target.wrist_output = wrist_.readings().wrist_position;
    lastWristManual = false;
  }

  pivot_.SetTarget(pivot_target);
  telescope_.SetTarget(telescope_target);
  wrist_.SetTarget(wrist_target);
}

bool TeleopPositioningCommand::IsFinished() { return false; }