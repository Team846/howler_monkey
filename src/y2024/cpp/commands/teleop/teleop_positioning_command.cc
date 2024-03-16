#include "commands/teleop/teleop_positioning_command.h"
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
  static constexpr double pivotToWrist = 20.5;
  static constexpr double pivotToWristOffset = -4.25;
  static constexpr double wristToFlywheels = 6.2;

  static constexpr double wristToIntake = 14.5;
  static constexpr double wristToIntakeOtherAngle = radians(36.5);

  static constexpr double pivotToGround = 16.25;

  static constexpr double pivotToCenter = 9.25;

  static constexpr double robotWidth = 28;

  static double pow(double base, int exponent) {
    for (int i = 1; i < exponent; i++) {
      base *= base;
    }
    return base;
  }
  
  public:
static bool withinBounds(CoordinatePositions pos) {
    return (pos.forward_axis < robotWidth / 2.0 + 11.5 && pos.forward_axis > -robotWidth / 2.0 - 11.5
      &&  pos.upward_axis > -10.0 && pos.upward_axis < 47.5);
  }

  static RawPositions toRaw(CoordinatePositions pos) {
    // if (!withinBounds(pos)) {
    //   double angle = atan2(pos.upward_axis, pos.forward_axis);

    //   double maxMagnitude = std::abs(std::min(robotWidth / 2.0 + 11.5, -robotWidth / 2.0 - 11.5) / cos(angle));
      

    //   double magnitude = std::min(std::max(0.0, std::min(std::max(0.0, std::hypot(pos.forward_axis, pos.upward_axis)), maxMagnitude)), 47.5);

    //   pos.forward_axis = magnitude * cos(angle);
    //   pos.upward_axis = magnitude * sin(angle);
    // }

    RawPositions raw{};

    double pivotForwardComponent = (pos.forward_axis - (wristToIntake)*std::cos(pos.shooting_angle)) 
      + pivotToCenter;

    double pivotUpwardComponent = (pos.upward_axis - (wristToIntake)*std::sin(pos.shooting_angle)) - pivotToGround;
    
    raw.extension = std::sqrt(pow(pivotForwardComponent, 2) + pow(pivotUpwardComponent, 2) - pow(pivotToWristOffset, 2)) 
        - pivotToWrist;

    double pivotDistanceHypotenuse = (std::sqrt(pow(pivotToWristOffset, 2) 
      + pow(pivotToWrist + raw.extension, 2)));


    double pivotTotalAngle = (std::atan2(pivotUpwardComponent, pivotForwardComponent));

    double pivotAddedAngle = (std::atan2(pivotToWristOffset, pivotToWrist + raw.extension));

    raw.pivot_angle =  (pivotTotalAngle - pivotAddedAngle + radians(17.0));

    raw.wrist_angle = -(((pos.shooting_angle - raw.pivot_angle + radians(17.0) - radians(47))) - wristToIntakeOtherAngle);

    return raw;
  }

  static CoordinatePositions toCoordinate(RawPositions pos) {
    CoordinatePositions coordinate{};

    pos.pivot_angle -= radians(17.0);
    pos.wrist_angle -= radians(49.0);
  
    double truePivotAngle = (pos.pivot_angle) + (std::atan2(pivotToWristOffset,
          pos.extension+pivotToWrist));


    double pivotDistanceHypotenuse = (std::sqrt(pow(pivotToWristOffset, 2) 
      + pow(pivotToWrist + pos.extension, 2)));

    coordinate.upward_axis = (pivotToGround + 
       pivotDistanceHypotenuse * std::sin(truePivotAngle)) + 
        wristToIntake * std::sin(pos.pivot_angle - pos.wrist_angle + wristToIntakeOtherAngle);

    coordinate.forward_axis = pivotDistanceHypotenuse * std::cos(truePivotAngle) -
      pivotToCenter + wristToIntake * std::cos(pos.pivot_angle - pos.wrist_angle + wristToIntakeOtherAngle);

    coordinate.shooting_angle = ((pos.pivot_angle) - pos.wrist_angle + wristToIntakeOtherAngle); 

    return coordinate;
  }

  static CoordinatePositions degree_toCoordinate(RawPositions pos) {
    return toCoordinate({radians(pos.pivot_angle), radians(pos.wrist_angle), pos.extension});
  }
};

class TrapCalculator {
  public:
    static std::vector<std::pair<frc846::util::Vector2D<units::inch_t>, units::degree_t>> interpolateTrapPoints(
        frc846::util::Vector2D<units::inch_t> starting_coordinate,
          frc846::util::Vector2D<units::inch_t> ending_coordinate, 
            units::degree_t starting_angle, units::degree_t ending_angle, int steps = 20) {
      std::vector<std::pair<frc846::util::Vector2D<units::inch_t>, units::degree_t>> toReturn{};
      for (int i = 0; i < steps; i++) {
        auto x_coord = starting_coordinate.x + (i/steps) * (ending_coordinate.x - starting_coordinate.x);
        auto y_coord = starting_coordinate.y + (i/steps) * (ending_coordinate.y - starting_coordinate.y);
        auto angle = starting_angle + (i/steps) * (ending_angle - starting_angle);
        toReturn.push_back({{x_coord, y_coord}, angle});

        std::cout << x_coord.to<double>() << "X" << y_coord.to<double>() << "Y" << angle.to<double>() << std::endl;
      }
      return toReturn;
    }

    static RawPositions getRawsAtPoint(int counter, 
      std::vector<std::pair<frc846::util::Vector2D<units::inch_t>, units::degree_t>> trapPoints) {
        return InverseKinematics::toRaw(CoordinatePositions{radians(trapPoints.at(counter).second.to<double>()), trapPoints.at(counter).first.x.to<double>(), 
          trapPoints.at(counter).first.y.to<double>()});
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
      bracer_(container.bracer_),
      super_(container.super_structure_) {
  AddRequirements({&pivot_, &telescope_, &wrist_, &bracer_});
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

  bool running_zero_bubble_position = operator_.readings().back_button;
  bool running_pass_position = driver_.readings().a_button;

  bool running_source = driver_.readings().x_button;

  bool pre_climb = operator_.readings().left_trigger;
  bool climb = operator_.readings().right_trigger;

  bool ftrap_s = operator_.readings().left_bumper;

  PivotTarget pivot_target = pivot_.ZeroTarget();
  TelescopeTarget telescope_target = telescope_.ZeroTarget();
  WristTarget wrist_target = wrist_.ZeroTarget();   

  if (pivot_up_manual || pivot_down_manual) {
    if (pivot_up_manual) {
      mpiv_adj += 30.0 / 50.0;
      ms_adj += 40.0 / 50.0;
    } else if (pivot_down_manual) {
      mpiv_adj -= 30.0 / 50.0;
      ms_adj -= 40.0 / 50.0;
    }
  }

  RawPositions positions{};
  
  if (ftrap_s) {
    auto k = TrapCalculator::interpolateTrapPoints(
        {super_.trap_start_x.value(), super_.trap_start_y.value()},
        {super_.trap_end_x.value(), super_.trap_end_y.value()},
        super_.trap_start_angle.value(),
        super_.trap_end_angle.value());
    positions = TrapCalculator::getRawsAtPoint(std::min(19, trapCounter/trapDivisor), k);
    std::cout << trapCounter << "Q" << std::min(19, trapCounter/trapDivisor) << std::endl;
    trapCounter += 1;
  }

  if (telescope_in_manual || telescope_out_manual) {
    if (telescope_in_manual) {
      mtele_adj -= 6.0 / (2.0 * 50.0);
    } else if (telescope_out_manual) {
      mtele_adj += 6.0 / (2.0 * 50.0);
    }
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
      ms_adj -= 40.0 / 50.0;
    } else if (w_d) {
      ms_adj += 40.0 / 50.0;
    }
  }

  frc846::util::ShareTables::SetString("shooting_state", "kNone");

  if (ftrap_s) {
    pivot_target.pivot_output = units::degree_t(degs(positions.pivot_angle));

    pivotHasRun = true;
  } else if (climb) {
    pivot_target.pivot_output = units::degree_t(setpoints::kClimb(0));

    std::cout << setpoints::kClimb(0) << std::endl;

    pivotHasRun = true;
  } else if (pre_climb) {
    bracer_.SetTarget(bracer_.MakeTarget(BracerState::kExtend));

    pivot_target.pivot_output = units::degree_t(setpoints::kPreClimb(0));

    std::cout << setpoints::kPreClimb(0) << std::endl;

    pivotHasRun = true;
  } else if (running_prep_speaker && frc846::util::ShareTables::GetDouble("telescope_extension") < 1.0) {
    double nextPivotTarget = setpoints::kShoot(0);
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);                                                                                                                                                                                                                                                                                                                                                                         ;

    pivotHasRun = true;
  } else if (running_source) {
    double nextPivotTarget = setpoints::kSource(0);
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    pivotHasRun = true;
  } else if (running_intake_position) {
    double nextPivotTarget = setpoints::kIntake(0);
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    pivotHasRun = true;
  } else if (running_amp_position) {
    double nextPivotTarget = setpoints::kAmp(0);
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    pivotHasRun = true;
  } else if (running_pass_position) {
    double nextPivotTarget = setpoints::kIntake(0);
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    pivotHasRun = true;
  } else if (running_zero_bubble_position) {
    double nextPivotTarget = setpoints::kZeroBubble(0);
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    pivotHasRun = true;
  } else if (pivotHasRun) {
    double nextPivotTarget = setpoints::kStow(0);
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

    mu_adj = 0.0;
    mx_adj = 0.0;
    ms_adj = 0.0;

    mpiv_adj = 0.0;

    pivotHasRun = false;
  } else {
    double nextPivotTarget = setpoints::kStow(0);
    pivot_target.pivot_output = units::degree_t(nextPivotTarget);

  }

  if (ftrap_s) {
    telescope_target.extension = units::inch_t(positions.extension);

    telescopeHasRun = true;
  } else if (climb || pre_climb) {
    telescope_target.extension = units::inch_t(0);

    telescopeHasRun = true;
  } else if (running_prep_speaker) {
    double nextTelescopeTarget = setpoints::kShoot(1);
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    telescopeHasRun = true;
  } else if (running_source) {
    double nextTelescopeTarget = setpoints::kSource(1);
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    telescopeHasRun = true;
  } else if (running_intake_position && frc846::util::ShareTables::GetDouble("pivot_position") < 20.0) {
    double nextTelescopeTarget = setpoints::kIntake(1);
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    telescopeHasRun = true;
  } else if (running_amp_position) {
    double nextTelescopeTarget = setpoints::kAmp(1);
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    telescopeHasRun = true;
  } else if (running_zero_bubble_position) {
    double nextTelescopeTarget = setpoints::kZeroBubble(1);
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    telescopeHasRun = true;
  } else if (running_pass_position) {
    double nextTelescopeTarget = setpoints::kIntake(1);
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    telescopeHasRun = true;
  } else if (telescopeHasRun) {
    double nextTelescopeTarget = setpoints::kStow(1);
    telescope_target.extension = units::inch_t(nextTelescopeTarget);

    mtele_adj = 0.0;

    telescopeHasRun = false;
  } else {
    double nextTelescopeTarget = setpoints::kStow(1);
    telescope_target.extension = units::inch_t(nextTelescopeTarget);
  }

  if (ftrap_s) {
    wrist_target.wrist_output = units::degree_t(degs(positions.wrist_angle));

    telescopeHasRun = true;
  } else if (climb || pre_climb) {
    double nextWristTarget = setpoints::kStow(2);
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    wristHasRun = true;
  } else if (running_prep_speaker && frc846::util::ShareTables::GetDouble("pivot_position") > 2.0) {
    double shooting_dist = (field::points::kSpeakerTeleop(!frc846::util::ShareTables::GetBoolean("is_red_side")) - drivetrain_.readings().pose.point).Magnitude().to<double>();

    // auto robot_velocity = drivetrain_.readings().velocity;
    // auto point_target = (field::points::kSpeakerTeleop(!frc846::util::ShareTables::GetBoolean("is_red_side")) - drivetrain_.readings().pose.point);

    // double robot_velocity_in_component = 
    //   (robot_velocity.x.to<double>() * point_target.x.to<double>() + 
    //     robot_velocity.y.to<double>() * point_target.y.to<double>())/point_target.Magnitude().to<double>();


    // double robot_velocity_orth_component = std::sqrt(robot_velocity.Magnitude().to<double>()*
    //   robot_velocity.Magnitude().to<double>() - robot_velocity_in_component * robot_velocity_in_component);


    shooting_dist = 3.4; //REMOVE, FIX

    shooting_dist =  shooting_dist + super_.teleop_shooter_x_.value().to<double>()/12.0; //(13.0 + 40.0) / 12.0;

    units::degree_t theta = units::degree_t(TeleopShootingCalculator::calculate(scorer_.shooting_exit_velocity_.value(), shooting_dist - 2.6, 
      0.0, 0.0, super_.teleop_shooter_height_.value().to<double>(), super_.shoot_angle_calc_intial_guess_.value().to<double>(), super_.shoot_angle_calc_tolerance_.value(), super_.shoot_angle_calc_max_iterations_.value()));

    std::cout << shooting_dist << "SD" << theta.to<double>() << std::endl;
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
  } else if (running_source && frc846::util::ShareTables::GetDouble("pivot_position") > 5.0
    &&  frc846::util::ShareTables::GetDouble("telescope_extension") < 1.0) {
    double nextWristTarget = setpoints::kSource(2);
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    wristHasRun = true;
  } else if (running_intake_position && frc846::util::ShareTables::GetDouble("pivot_position") < 20.0) {
    double nextWristTarget = setpoints::kIntake(2);
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    wristHasRun = true;
  } else if (running_amp_position) {
    double nextWristTarget = setpoints::kAmp(2);
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    wristHasRun = true;
  } else if (running_pass_position) {
    double nextWristTarget = setpoints::kIntake(2);
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    wristHasRun = true;
  } else if (running_zero_bubble_position) {
    double nextWristTarget = setpoints::kZeroBubble(2);
    wrist_target.wrist_output = units::degree_t(nextWristTarget);

    wristHasRun = true;
  } else if (wristHasRun) {
    double nextWristTarget = setpoints::kStow(2);
    wrist_target.wrist_output = units::degree_t(nextWristTarget);
    wristHasRun = false;
  } else {
    double nextWristTarget = setpoints::kStow(2);
    wrist_target.wrist_output = units::degree_t(nextWristTarget);
  }


  if (auto* piv_pos = std::get_if<units::angle::degree_t>(&pivot_target.pivot_output)) {
    if (auto* wrist_pos = std::get_if<units::angle::degree_t>(&wrist_target.wrist_output)) {
      if (auto* tele_ext = std::get_if<units::inch_t>(&telescope_target.extension)) {
        CoordinatePositions coordinateConverted = InverseKinematics::degree_toCoordinate({piv_pos->to<double>() + mpiv_adj, 
          wrist_pos->to<double>() + ms_adj, tele_ext->to<double>() + mtele_adj});

        // std::cout << "FA" << coordinateConverted.forward_axis << std::endl;
        // std::cout << coordinateConverted.upward_axis << std::endl;

        if (!InverseKinematics::withinBounds(coordinateConverted)) {
          mpiv_adj = pmpiv_adj;
          ms_adj = pms_adj;
          mtele_adj = pmtele_adj;

          std::cout << "OUT of bounds, forward " << coordinateConverted.forward_axis << ", upward" << coordinateConverted.upward_axis << std::endl;
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

  if (operator_.readings().start_button) {
    bracer_.SetTarget(bracer_.MakeTarget(BracerState::kRetract));
  }

  pivot_.SetTarget(pivot_target);
  telescope_.SetTarget(telescope_target);
  wrist_.SetTarget(wrist_target);
}

bool TeleopPositioningCommand::IsFinished() { return false; }