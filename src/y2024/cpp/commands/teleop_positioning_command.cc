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

    static constexpr double h = 81.0/12.0 - 39.0/12.0;
    static constexpr double l = 14.0 / 12.0;

    static constexpr double k = 0; //0.43;
    static constexpr double w = 0; //105.0 * pi / 180.0;

  public:

  static double f_of_x(double v, double x, double d, double r_v, double r_o) {
    double cosx = cos(x);
    double sinx = sin(x);
    return ((v*sinx)*(d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v)
                -1.0/2.0*g*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))  
                    +l*sinx/2+k*sin(x+w)-h);
  }

  static double f_prime(double v, double x, double d, double r_v, double r_o) {
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

  static double calculate(double v, double d, double r_v, double r_o, 
    double initial_guess = radians(1.01), double tolerance=0.06, double max_iterations=5000) {
    double x = initial_guess;
    for (int i = 0; i < max_iterations; i++) {
        auto fx = f_of_x(v, x, d, r_v, r_o);
        if (std::abs(fx) < tolerance) {
          return degs(x);
        }

        x -= std::min(radians(140.0/std::max(18, (i+1))), 
          std::max(radians(-140.0/std::max(18, (i+1))), (fx / f_prime(v, x, d, r_v, r_o))));

        if (x < 0.0) x = radians(1.01);
        else if (x > pi / 2) x = radians(1.01);
    }

    return 0.0;
  }
};

// struct RawPositions {
//   double pivot_angle;
//   double wrist_angle;
//   double extension;
// };

// struct CoordinatePositions {
//   double shooting_angle;
//   double forward_axis;
//   double upward_axis;
// };

static double radians(double degs) {
  return degs * 3.141592658979 / 180;
}

static double degs(double radians) {
    return radians * 180 / 3.141592658979;
}



TeleopPositioningCommand::TeleopPositioningCommand(RobotContainer& container)
    : driver_(container.driver_),
      operator_(container.operator_),
      drivetrain_(container.drivetrain_),
      pivot_(container.pivot_),
      telescope_(container.telescope_),
      wrist_(container.wrist_),
      scorer_(container.scorer_) {
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
    pivot_target.pivot_output = -3_deg;

    pivotHasRun = true;
  } else if (pre_climb) {
    pivot_target.pivot_output = 100_deg;

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


    shooting_dist =  shooting_dist + (4.0) / 12.0; //(13.0 + 40.0) / 12.0;

    units::degree_t theta = units::degree_t(TeleopShootingCalculator::calculate(scorer_.shooting_exit_velocity_.value(), shooting_dist - 2.6, 
      0.0, 0.0));


    if (shooting_dist < 11.0 && std::abs(scorer_.readings().kLeftErrorPercent) < 0.2) {
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

  // if (auto* piv_pos = std::get_if<units::angle::degree_t>(&pivot_target.pivot_output)) {
  //   if (auto* wrist_pos = std::get_if<units::angle::degree_t>(&wrist_target.wrist_output)) {
  //     if (auto* tele_ext = std::get_if<units::inch_t>(&telescope_target.extension)) {
  //       CoordinatePositions coordinateConverted = InverseKinematics::toCoordinate({radians(piv_pos->to<double>()), 
  //         radians(wrist_pos->to<double>()), tele_ext->to<double>()});
  //       coordinateConverted.forward_axis += mx_adj;// - pmx_adj;
  //       coordinateConverted.upward_axis += mu_adj;// - pmu_adj;
  //       // coordinateConverted.shooting_angle += radians(ms_adj);

  //       pivot_target.pivot_output = units::degree_t(degs(InverseKinematics::toRaw(coordinateConverted).pivot_angle) + mpiv_adj);
  //       wrist_target.wrist_output = units::degree_t(degs(InverseKinematics::toRaw(coordinateConverted).wrist_angle) + ms_adj);
  //       telescope_target.extension = units::inch_t(InverseKinematics::toRaw(coordinateConverted).extension + mtele_adj);
  //     } 
  //   }
  // }

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