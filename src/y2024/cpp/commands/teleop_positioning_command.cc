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

    static constexpr double h_speaker = 81.0/12.0;
    static constexpr double l = 14.0 / 12.0;

    static constexpr double k = 0; //0.43;
    static constexpr double w = 0; //105.0 * pi / 180.0;

  public:

  static double f_of_x(double v, double x, double d, double r_v, double r_o, double h_shooter) {
    double h=h_speaker-h_shooter/12.0;
    double cosx = cos(x);
    double sinx = sin(x);
    return ((v*sinx)*(d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v)
                -1.0/2.0*g*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))  
                    +l*sinx/2+k*sin(x+w)-h);
  }

  static double f_prime(double v, double x, double d, double r_v, double r_o, double h_shooter)  {
    double h=h_speaker-h_shooter/12.0;
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

  static double calculate(double v, double d, double r_v, double r_o, double h_shooter,
    double initial_guess = radians(1.01), double tolerance=0.06, double max_iterations=5000) {
    double h = h_speaker-h_shooter/12.0;
    double x = initial_guess;
    for (int i = 0; i < max_iterations; i++) {
        auto fx = f_of_x(v, x, d, r_v, r_o, h_shooter);
        if (std::abs(fx) < tolerance) {
          return degs(x);
        }

        x -= std::min(radians(140.0/std::max(18, (i+1))), 
          std::max(radians(-140.0/std::max(18, (i+1))), (fx / f_prime(v, x, d, r_v, r_o, h_shooter))));

        if (x < 0.0) x = radians(initial_guess);
        else if (x > pi / 2) x = radians(initial_guess);
    }

    return 0.0;
  }
};

struct RawPositions {
  double pivot_angle;
  double wrist_angle;
  double extension;
};

struct CoordinatePositions {
  double shooting_angle;
  double forward_axis;
  double upward_axis;
};

static constexpr double radians(double degs) {
  return degs * 3.141592658979 / 180;
}

static constexpr double degs(double radians) {
    return radians * 180 / 3.141592658979;
}

class InverseKinematics {
  private:
  static constexpr double pivotToWrist = 20.05;
  static constexpr double pivotToWristOffset = -4.25;
  static constexpr double wristToFlywheels = 6.2;

  static constexpr double pivotToGround = 17.5;

  static constexpr double pivotToCenter = 9.5; //FIX

  static constexpr double robotWidth = 28;

  static double pow(double base, int exponent) {
    for (int i = 1; i < exponent; i++) {
      base *= base;
    }
    return base;
  }
  
  public:
  static bool withinBounds(CoordinatePositions pos) {
    return (pos.forward_axis > robotWidth / 2.0 + 10.0 || pos.forward_axis < -robotWidth / 2.0 - 10.0
      ||  pos.upward_axis < 0.0 || pos.upward_axis > 47.0);
  }

  static RawPositions toRaw(CoordinatePositions pos) {
    if (pos.forward_axis > robotWidth / 2.0 + 10.0 || pos.forward_axis < -robotWidth / 2.0 - 10.0
      ||  pos.upward_axis < 0.0 || pos.upward_axis > 47.0) {

      double angle = atan2(pos.upward_axis, pos.forward_axis);

      double maxMagnitude = std::abs(std::min(robotWidth / 2.0 + 10.0, -robotWidth / 2.0 - 10.0) / cos(angle));
      

      double magnitude = std::min(std::max(0.0, std::min(std::max(0.0, std::hypot(pos.forward_axis, pos.upward_axis)), maxMagnitude)), 47.0);

      pos.forward_axis = magnitude * cos(angle);
      pos.upward_axis = magnitude * sin(angle);
    }

    RawPositions raw{};

    double pivotForwardComponent = (pos.forward_axis - (wristToFlywheels)*std::cos(pos.shooting_angle)) 
      - pivotToCenter;

    double pivotUpwardComponent = (pos.upward_axis - (wristToFlywheels)*std::sin(pos.shooting_angle)) - pivotToGround;
    
    raw.extension = std::sqrt(pow(pivotForwardComponent, 2) + pow(pivotUpwardComponent, 2) - pow(pivotToWristOffset, 2)) 
        - pivotToWrist;

    double pivotDistanceHypotenuse = (std::sqrt(pow(pivotToWristOffset, 2) 
      + pow(pivotToWrist + raw.extension, 2)));


    double pivotTotalAngle = (std::atan2(pivotForwardComponent, pivotUpwardComponent));

    double pivotAddedAngle = (std::atan2(pivotToWristOffset, pivotToWrist + raw.extension));

    raw.pivot_angle =  radians(90) - (pivotTotalAngle - pivotAddedAngle - radians(17.0));

    raw.wrist_angle = -(pos.shooting_angle - (std::atan2(pivotUpwardComponent, pivotForwardComponent)) - radians(47.0));

    return raw;
  }

  static CoordinatePositions toCoordinate(RawPositions pos) {
    CoordinatePositions coordinate{};

    pos.pivot_angle -= radians(17.0);
    pos.wrist_angle -= radians(47.0);
  
    double truePivotAngle = (pos.pivot_angle) - (std::atan2(pivotToWristOffset,
          pos.extension+pivotToWrist));


    double pivotDistanceHypotenuse = (std::sqrt(pow(pivotToWristOffset, 2) 
      + pow(pivotToWrist + pos.extension, 2)));

    coordinate.upward_axis = (pivotToGround + 
       pivotDistanceHypotenuse * std::sin(truePivotAngle) + 
        wristToFlywheels * std::sin(truePivotAngle - pos.wrist_angle));

    coordinate.forward_axis = (pivotDistanceHypotenuse * std::cos(truePivotAngle) + 
      wristToFlywheels * std::cos(truePivotAngle - pos.wrist_angle)) + pivotToCenter;

    coordinate.shooting_angle = ((truePivotAngle) - pos.wrist_angle); 

    return coordinate;
  }

  static CoordinatePositions degree_toCoordinate(RawPositions pos) {
    return toCoordinate({radians(pos.pivot_angle), radians(pos.wrist_angle), pos.extension});
  }
};

class TrapCalculator{
  private:
    static constexpr units::inch_t pivotHookDist=10_in;
    static constexpr units::degree_t pivotHookAngle=10_deg;
    static constexpr units::inch_t pivotToWrist=10_in;
    static constexpr units::inch_t wristToUpperIntake=10_in;
    static constexpr units::degree_t wristToUpperIntakeOffset=10_deg;
    static constexpr units::inch_t wristToLowerIntake=10_in;
    static constexpr units::inch_t pivotToWristOffset=-4.25_in;
    static constexpr units::inch_t chainToStageX=10_in;

  public:
    static units::degree_t getWristAngle(units::degree_t pivotAngle, units::inch_t telescope_extension, units::inch_t drivetrain_tilt, units::degree_t wrist_guess) { //drivetrain_tilt could be constnat, depends on how climb works
      // pivotAngle-=17_deg;

      // units::degree_t truePivotAngle = ((pivotAngle) + (units::math::atan2(telescope_extension+pivotToWrist, pivotToWristOffset)))-90_deg;

      // units::inch_t pivotDistanceHypotenuse = (units::math::sqrt(units::math::pow<2>(pivotToWristOffset) 
      //   + units::math::pow<2>(pivotToWrist+telescope_extension) ));

      // frc846::util::Vector2D<units::inch_t> wristPos; 
      // wristPos.x=-pivotHookDist*units::math::cos(drivetrain_tilt)*units::math::cos(truePivotAngle-drivetrain_tilt); 
      // wristPos.y=pivotHookDist*units::math::sin(drivetrain_tilt)*pivotDistanceHypotenuse*units::math::sin(truePivotAngle-drivetrain_tilt);

      // units::degree_t topRollerSolution=180_deg+(truePivotAngle-drivetrain_tilt+(-units::math::acos((chainToStageX-wristPos.x)/wristToUpperIntake)+10_deg))-133_deg;      
      // units::degree_t bottomRollerSolution=180_deg + (truePivotAngle-drivetrain_tilt+units::math::acos((chainToStageX-wristPos.x)/wristToLowerIntake))-133_deg;

      // if (units::math::abs(topRollerSolution-wrist_guess)<units::math::abs(bottomRollerSolution-wrist_guess)){
      //   return topRollerSolution;
      // }
      // else{
      //   return bottomRollerSolution;
      // }
      return 0_deg;
    }
};

TeleopPositioningCommand::TeleopPositioningCommand(RobotContainer& container)
    : driver_(container.driver_),
      operator_(container.operator_),
      drivetrain_(container.drivetrain_),
      pivot_(container.pivot_),
      telescope_(container.telescope_),
      wrist_(container.wrist_),
      scorer_(container.scorer_), 
      super_(container.super_structure_) {
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

  bool running_intake_position = driver_.readings().left_trigger;
  bool running_amp_position = driver_.readings().left_bumper;
  bool running_prep_speaker = driver_.readings().right_trigger || driver_.readings().y_button;

  bool pre_climb = operator_.readings().left_trigger;
  bool climb = operator_.readings().right_trigger;

  bool ftrap_s = operator_.readings().start_button;

  PivotTarget pivot_target = pivot_.ZeroTarget();
  TelescopeTarget telescope_target = telescope_.ZeroTarget();
  WristTarget wrist_target = wrist_.ZeroTarget();   

  if (pivot_up_manual || pivot_down_manual) {
    if (pivot_up_manual) {
      mpiv_adj += 40.0 / 50.0;
    } else if (pivot_down_manual) {
      mpiv_adj -= 40.0 / 50.0;
    }
  }

  if (telescope_in_manual || telescope_out_manual) {
    if (telescope_in_manual) {
      mtele_adj -= 6.0 / (2.0 * 50.0);
    } else if (telescope_out_manual) {
      mtele_adj += 6.0 / (2.0 * 50.0);
    }

    mtele_adj = std::max(-4.0, std::min(4.0, mtele_adj));
  }

  // double translate_x = frc846::util::HorizontalDeadband(
  //     operator_.readings().left_stick_x, driver_.translation_deadband_.value(), 1,
  //     driver_.translation_exponent_.value(), 1);
  // double translate_u = frc846::util::HorizontalDeadband(
  //     operator_.readings().left_stick_y, driver_.translation_deadband_.value(), 1,
  //     driver_.translation_exponent_.value(), 1);

  // if (std::abs(translate_x) > 0.02) {
  //   mx_adj -= 11.0 / 50.0 * translate_x;
  //   mx_adj = std::max(std::min(mx_adj, 28.0 + 12.0), -(28.0 + 12.0));
  // }

  // if (std::abs(translate_u) > 0.02) {
  //   mu_adj += 11.0 / 50.0 * translate_u;
  //   mu_adj = std::min(std::max(mu_adj, -48.0), 48.0);
  // }

  bool w_up = operator_.readings().pov == frc846::XboxPOV::kUp;
  bool w_d = operator_.readings().pov == frc846::XboxPOV::kDown;

  if (w_up || w_d) {
    if (w_up) {
      ms_adj += 40.0 / 50.0;
    } else if (w_d) {
      ms_adj -= 40.0 / 50.0;
    }
  }

  frc846::util::ShareTables::SetString("shooting_state", "kNone");

  if (ftrap_s) {
    double nextPivotTarget = setpoints::kTrap(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    pivotHasRun = true;
  } else if (climb) {
    pivot_target.pivot_output = setpoints::kClimb(0).value();

    pivotHasRun = true;
  } else if (pre_climb) {
    pivot_target.pivot_output = setpoints::kPreClimb(0).value();

    pivotHasRun = true;
  } else if (running_prep_speaker) {
    double nextPivotTarget = setpoints::kShoot(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);                                                                                                                                                                                                                                                                                                                                                                         ;

    pivotHasRun = true;
  } else if (running_intake_position) {
    double nextPivotTarget = setpoints::kIntake(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    pivotHasRun = true;
  } else if (running_amp_position) {
    double nextPivotTarget = setpoints::kAmp(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    pivotHasRun = true;
  } else if (pivotHasRun) {
    double nextPivotTarget = setpoints::kStow(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    mu_adj = 0.0;
    mx_adj = 0.0;
    ms_adj = 0.0;

    mpiv_adj = 0.0;

    pivotHasRun = false;
  } else {
    double nextPivotTarget = setpoints::kStow(0).value();
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);
  }

  if (ftrap_s) {
    double nextTelescopeTarget = setpoints::kTrap(1).value();
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    telescopeHasRun = true;
  } else if (climb || pre_climb) {
    telescope_target.extension = units::inch_t(0);

    telescopeHasRun = true;
  } else if (running_prep_speaker) {
    double nextTelescopeTarget = setpoints::kShoot(1).value();
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

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

    mtele_adj = 0.0;

    telescopeHasRun = false;
  } else {
    double nextTelescopeTarget = setpoints::kStow(1).value();
    telescope_target.extension = units::inch_t(nextTelescopeTarget);
  }

  if (ftrap_s) {
    double nextWristTarget = setpoints::kTrap(2).value();
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    telescopeHasRun = true;
  } else if (climb || pre_climb) {
    double nextWristTarget = setpoints::kStow(2).value();
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    wristHasRun = true;
  } else if (running_prep_speaker) {
    double shooting_dist = (field::points::kSpeakerTeleop(!frc846::util::ShareTables::GetBoolean("is_red_side")) - drivetrain_.readings().pose.point).Magnitude().to<double>();

    auto robot_velocity = drivetrain_.readings().velocity;
    auto point_target = (field::points::kSpeakerTeleop(!frc846::util::ShareTables::GetBoolean("is_red_side")) - drivetrain_.readings().pose.point);

    double robot_velocity_in_component = 
      (robot_velocity.x.to<double>() * point_target.x.to<double>() + 
        robot_velocity.y.to<double>() * point_target.y.to<double>())/point_target.Magnitude().to<double>();


    double robot_velocity_orth_component = std::sqrt(robot_velocity.Magnitude().to<double>()*
      robot_velocity.Magnitude().to<double>() - robot_velocity_in_component * robot_velocity_in_component);


    shooting_dist =  shooting_dist + super_.teleop_shooter_x_.value().to<double>()/12.0; //(13.0 + 40.0) / 12.0;

    units::degree_t theta = units::degree_t(TeleopShootingCalculator::calculate(scorer_.shooting_exit_velocity_.value(), shooting_dist - 2.6, 
      0.0, 0.0, super_.teleop_shooter_height_.value().to<double>(), super_.shoot_angle_calc_intial_guess_.value().to<double>(), super_.shoot_angle_calc_tolerance_.value(), super_.shoot_angle_calc_max_iterations_.value()));


    if (shooting_dist < super_.shooter_range_.value().to<double>() && std::abs(scorer_.readings().kLeftErrorPercent) < super_.shooter_speed_tolerance_.value()) {
      DriverTarget driver_target{true};
      driver_.SetTarget(driver_target);
      frc846::util::ShareTables::SetString("shooting_state", "kReady");
    } else {
      DriverTarget driver_target{false};
      driver_.SetTarget(driver_target);
      frc846::util::ShareTables::SetString("shooting_state", "kUnready");
    }

    if (!driver_.readings().y_button) {
      wrist_target.wrist_output = 90_deg + units::degree_t(frc846::util::ShareTables::GetDouble("pivot_position")) - 17.0_deg
        - wrist_.wrist_home_offset_.value() + units::degree_t(theta); //units::degree_t(setpoints::point_blank_wrist_.value());
    } else {
      wrist_target.wrist_output = 90_deg + units::degree_t(frc846::util::ShareTables::GetDouble("pivot_position")) - 17.0_deg
        - wrist_.wrist_home_offset_.value() + units::degree_t(setpoints::point_blank_wrist_.value());
    }

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
    wristHasRun = false;
  } else {
    double nextWristTarget = setpoints::kStow(2).value();
    wrist_target.wrist_output = units::degree_t(nextWristTarget);
  }


  if (auto* piv_pos = std::get_if<units::angle::degree_t>(&pivot_target.pivot_output)) {
    if (auto* wrist_pos = std::get_if<units::angle::degree_t>(&wrist_target.wrist_output)) {
      if (auto* tele_ext = std::get_if<units::inch_t>(&telescope_target.extension)) {
        CoordinatePositions coordinateConverted = InverseKinematics::toCoordinate({radians(piv_pos->to<double>() + mpiv_adj), 
          radians(wrist_pos->to<double>() + ms_adj), tele_ext->to<double>() + mtele_adj});

        if (!InverseKinematics::withinBounds(coordinateConverted)) {
          mpiv_adj = pmpiv_adj;
          ms_adj = pms_adj;
          mtele_adj = pmtele_adj;

          std::cout << "OUT of bounds" << std::endl;
        } else {
          pmpiv_adj =  mpiv_adj;
          pms_adj = ms_adj;
          pmtele_adj = mtele_adj;
        }
      } 
    }
  }

  if (auto* piv_pos = std::get_if<units::angle::degree_t>(&pivot_target.pivot_output)) {
    pivot_target.pivot_output = *piv_pos + units::degree_t(mpiv_adj);
  }

  if (auto* tele_ext = std::get_if<units::inch_t>(&telescope_target.extension)) {
    telescope_target.extension = *tele_ext + units::inch_t(mtele_adj);
  }

  if (auto* w_pos = std::get_if<units::angle::degree_t>(&wrist_target.wrist_output)) {
    wrist_target.wrist_output = *w_pos + units::degree_t(ms_adj);
  }

  if (!scorer_.GetHasPiece()) {
    DriverTarget driver_target{false};
    driver_.SetTarget(driver_target);
  }

  pivot_.SetTarget(pivot_target);
  telescope_.SetTarget(telescope_target);
  wrist_.SetTarget(wrist_target);
}

bool TeleopPositioningCommand::IsFinished() { return false; }