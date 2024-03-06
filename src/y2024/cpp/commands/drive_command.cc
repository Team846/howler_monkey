#include "commands/drive_command.h"

#include <utility>

#include "frc846/util/math.h"
#include "subsystems/field.h"
#include "subsystems/drivetrain.h"
#include "subsystems/swerve_module.h"
#include "frc846/loggable.h"
#include "frc846/util/share_tables.h"

class DriveShootingCalculator {
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

    static constexpr double h_speaker = 81.0/12;
    static constexpr double l = 14.0 / 12.0;

    static constexpr double v = 48.0;

    static constexpr double k = 0; //0.43;
    static constexpr double w = 0; //105.0 * pi / 180.0;

  public:

  static double f_of_x(double x, double d, double r_v, double r_o, double h_shooter) {
    double h = h_speaker-h_shooter/12.0;
    double cosx = cos(x);
    double sinx = sin(x);
    return ((v*sinx)*(d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v)
                -1.0/2.0*g*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))  
                    +l*sinx/2+k*sin(x+w)-h);
  }

  static double f_prime(double x, double d, double r_v, double r_o, double h_shooter) {
    double h = h_speaker-h_shooter/12.0;
    double cosx = cos(x);
    double sinx = sin(x);

    double t = 0.0;
    double t_prime = 0.0;

    try {
      t = (d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/v*cosx))*(r_o/v*cosx)+r_v);
      t_prime = (l*r_v*v*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v)) + l*r_o*r_o*cosx*cosx*cosx - 4*d*r_o*r_o*cosx*cosx + 2*d*v*v)*sinx /
                      (2*v*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v))*(r_v+v*cosx*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v))));
    } catch (std::exception exc) {}

    return v*cosx*t + v*sinx*t_prime + l*cosx/2 - 1.0/2.0*g*2*t*t_prime;
  }

  static double calculate(double d, double r_v, double r_o, double h_shooter,
    double initial_guess = radians(1.01), double tolerance=0.04, double max_iterations=600) {
    double h = h_speaker-h_shooter/12.0;
    double x = initial_guess;
    for (int i = 0; i < max_iterations; i++) {
        auto fx = f_of_x(x, d, r_v, r_o, h_shooter);
        if (std::abs(fx) < tolerance) {
          return degs(std::asin(r_o / (v * std::cos(x))));
        }

        x -= std::min(radians(120.0/std::max(14, (i+1))), 
          std::max(radians(-120.0/std::max(14, (i+1))), fx / f_prime(x, d, r_v, r_o, h_shooter))) / 10.0;

        if (x < 0.0) x = radians(initial_guess);
        else if (x > pi / 2) x = radians(initial_guess);
    }

    return 0.0;
  }
};

DriveCommand::DriveCommand(RobotContainer& container)
    : driver_(container.driver_),
      drivetrain_(container.drivetrain_), 
      super_(container.super_structure_) {
  AddRequirements({&drivetrain_});
  SetName("drive_command");
}

void DriveCommand::Execute() {
  DrivetrainTarget drivetrain_target;

  // -----BUTTON MAPPINGS-----

  // Left bumper   | robot centric translation
  // Right bumper  | precision drive

  bool is_robot_centric = false;
  bool is_slow_drive = driver_.readings().right_bumper;
  bool prep_align_speaker = driver_.readings().right_trigger;


  // -----TRANSLATION CONTROL-----

  double translate_x = frc846::util::HorizontalDeadband(
      driver_.readings().left_stick_x, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
  double translate_y = frc846::util::HorizontalDeadband(
      driver_.readings().left_stick_y, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
      
  if (prep_align_speaker) {
    translate_x = units::math::min(0.75, units::math::max(translate_x, -0.75));
    translate_y = units::math::min(0.75, units::math::max(translate_y, -0.75));
  }

  drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value();
  drivetrain_target.v_y = translate_y * drivetrain_.max_speed_.value();

  // Slow down translation if slow mode is active
  if (is_slow_drive) {
    translate_x = frc846::util::HorizontalDeadband(
      driver_.readings().left_stick_x*abs(driver_.readings().left_stick_x), driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
    translate_y = frc846::util::HorizontalDeadband(
      driver_.readings().left_stick_y*abs(driver_.readings().left_stick_y), driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
    drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value() * drivetrain_.slow_mode_percent_.value();
    drivetrain_target.v_y = (prep_align_speaker ? 0.0 : translate_y) * drivetrain_.max_speed_.value() * drivetrain_.slow_mode_percent_.value();
  }

  // Robot vs field oriented translation
  drivetrain_target.translation_reference =
      (is_robot_centric) ? DrivetrainTranslationReference::kRobot
                       : DrivetrainTranslationReference::kField;

  drivetrain_target.control = kOpenLoop;

  // -----STEER CONTROL-----

  double steer_x = frc846::util::HorizontalDeadband(
      driver_.readings().right_stick_x, driver_.steer_deadband_.value(), 1,
      driver_.steer_exponent_.value(), 1);

  if (steer_x != 0) {
    // Manual steer
    auto target = steer_x * drivetrain_.max_omega();

    // Slow down steering if slow mode is active
    if (is_slow_drive) {
      target *= drivetrain_.slow_omega_percent_.value();
    }

    drivetrain_target.rotation = DrivetrainRotationVelocity(target);
  } else {
    // Hold position
    drivetrain_target.rotation = DrivetrainRotationVelocity(0_deg_per_s);
  }

  if (prep_align_speaker) {
    auto current_x = drivetrain_.readings().pose.point.x;
    auto current_y = drivetrain_.readings().pose.point.y;
    auto target_x = field::points::kSpeakerTeleop(!frc846::util::ShareTables::GetBoolean("is_red_side")).x;
    auto target_y = field::points::kSpeakerTeleop(!frc846::util::ShareTables::GetBoolean("is_red_side")).y;

    auto dist_x = target_x - current_x;
    auto dist_y = target_y - current_y;

    if (units::math::abs(dist_x) > 0.5_ft || units::math::abs(dist_y) > 0.5_ft) {
      auto target_angle = units::math::atan2(dist_x, units::math::abs(dist_y));

      double shooting_dist = (field::points::kSpeakerTeleop(!frc846::util::ShareTables::GetBoolean("is_red_side")) - drivetrain_.readings().pose.point).Magnitude().to<double>();

      auto robot_velocity = drivetrain_.readings().velocity;
      auto point_target = (field::points::kSpeaker() - drivetrain_.readings().pose.point);

      double robot_velocity_in_component = 
        (robot_velocity.x.to<double>() * point_target.x.to<double>() + 
          robot_velocity.y.to<double>() * point_target.y.to<double>())/point_target.Magnitude().to<double>();


      double robot_velocity_orth_component = std::sqrt(robot_velocity.Magnitude().to<double>()*
        robot_velocity.Magnitude().to<double>() - robot_velocity_in_component * robot_velocity_in_component);

      units::degree_t theta_adjust = 0_deg;//units::degree_t(DriveShootingCalculator::calculate(shooting_dist, 
        //0.0, 0.0, super_.teleop_shooter_height_.value().to<double>(), super_.shoot_drive_angle_calc_intial_guess_.value().to<double>(), super_.shoot_drive_angle_calc_tolerance_.value(), super_.shoot_drive_angle_calc_max_iterations_.value()));
   
      drivetrain_target.rotation = DrivetrainRotationPosition(target_angle - theta_adjust);
    }
  }


  drivetrain_.SetTarget(drivetrain_target);
}

bool DriveCommand::IsFinished() { return false; }