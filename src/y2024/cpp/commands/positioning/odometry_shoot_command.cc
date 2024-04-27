#include "commands/positioning/odometry_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

#include "subsystems/setpoints.h"
#include "subsystems/field.h"

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
    // double h=h_speaker-h_shooter/12.0;
    double cosx = cos(x);
    double sinx = sin(x);

    double t = 0.0;
    double t_prime = 0.0;

    try {
      t = (d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/v*cosx))*(r_o/v*cosx)+r_v);
      t_prime = (l*r_v*v*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v)) + l*r_o*r_o*cosx*cosx*cosx - 4*d*r_o*r_o*cosx*cosx + 2*d*v*v)*sinx /
                      (2*v*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v))*(r_v+v*cosx*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v))));
    } catch (std::exception const&) {
      std::cout << "Out of Range" << std::endl;
    }

    return v*cosx*t + v*sinx*t_prime + l*cosx/2 - 1.0/2.0*g*2*t*t_prime;
  }

  static double calculate(double v, double d, double r_v, double r_o, double h_shooter,
    double initial_guess = radians(1.01), double tolerance=0.06, double max_iterations=5000) {
    // double h = h_speaker-h_shooter/12.0;
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


OdometryShootCommand::OdometryShootCommand(
    RobotContainer& container)
    : frc846::Loggable{"odometry_shoot_command"}, pivot_(container.pivot_),
        telescope_(container.telescope_), wrist_(container.wrist_), operator_(container.operator_), drivetrain_(container.drivetrain_), super_(container.super_structure_), shooter_(container.shooter_), driver_(container.driver_) {
  AddRequirements({&pivot_, &telescope_, &wrist_, &drivetrain_, &driver_});
  SetName("odometry_shoot_command");
}

void OdometryShootCommand::Initialize() {
  Log("OdometryShoot Command Initialize");
}

void OdometryShootCommand::Execute() {

  if (frc846::util::ShareTables::GetString("arm_position")==""){
    frc846::util::ShareTables::SetString("arm_position", "odometry_shoot");
  }
  if (frc846::util::ShareTables::GetString("arm_position")=="odometry_shoot"){
    double shooting_dist = (field::points::kSpeakerTeleop(!frc846::util::ShareTables::GetBoolean("is_red_side")) - drivetrain_.readings().pose.point).Magnitude().to<double>();

    // auto robot_velocity = drivetrain_.readings().velocity;
    // auto point_target = (field::points::kSpeakerTeleop(!frc846::util::ShareTables::GetBoolean("is_red_side")) - drivetrain_.readings().pose.point);

    // double robot_velocity_in_component = 
    //   (robot_velocity.x.to<double>() * point_target.x.to<double>() + 
    //     robot_velocity.y.to<double>() * point_target.y.to<double>())/point_target.Magnitude().to<double>();


    // double robot_velocity_orth_component = std::sqrt(robot_velocity.Magnitude().to<double>()*
    //   robot_velocity.Magnitude().to<double>() - robot_velocity_in_component * robot_velocity_in_component);


    // shooting_dist = 3.4; //REMOVE, FIX

    shooting_dist += super_.teleop_shooter_x_.value().to<double>()/12.0; //(13.0 + 40.0) / 12.0;

    units::degree_t theta = units::degree_t(TeleopShootingCalculator::calculate(shooter_.shooting_exit_velocity_.value(), shooting_dist - 2.6, 
        0.0, 0.0, super_.teleop_shooter_height_.value().to<double>(), super_.shoot_angle_calc_intial_guess_.value().to<double>(), super_.shoot_angle_calc_tolerance_.value(), super_.shoot_angle_calc_max_iterations_.value()));

    if (shooting_dist < super_.shooter_range_.value().to<double>() && std::abs(shooter_.readings().kLeftErrorPercent) < super_.shooter_speed_tolerance_.value()) {
        DriverTarget driver_target{true};
        driver_.SetTarget(driver_target);
        frc846::util::ShareTables::SetString("shooting_state", "kReady");
    } else {
        DriverTarget driver_target{false};
        driver_.SetTarget(driver_target);
        frc846::util::ShareTables::SetString("shooting_state", "kUnready");
    }

    units::degree_t nextPivotTarget = units::degree_t(setpoints::kShoot(0));
    units::inch_t nextTelescopeTarget = units::inch_t(setpoints::kShoot(1));
    units::degree_t nextWristTarget = 90_deg + units::degree_t(frc846::util::ShareTables::GetDouble("pivot_position")) - 17.0_deg
            - wrist_.wrist_home_offset_.value() + units::degree_t(theta);;

    bool telescope_in_manual = operator_.readings().a_button;
    bool telescope_out_manual = operator_.readings().b_button;
    bool pivot_down_manual = operator_.readings().x_button;
    bool pivot_up_manual = operator_.readings().y_button;
    bool w_up = operator_.readings().pov == frc846::XboxPOV::kUp;
    bool w_d = operator_.readings().pov == frc846::XboxPOV::kDown;  

    if (w_up) {
        ms_adj -= wrist_.wrist_adj_inc_.value();
    } else if (w_d) {
        ms_adj += wrist_.wrist_adj_inc_.value();
    }

    if (telescope_in_manual) {
        mtele_adj -= telescope_.telescope_adj_inc_.value();
    } else if (telescope_out_manual) {
        mtele_adj += telescope_.telescope_adj_inc_.value();
    }

    if (pivot_up_manual) {
        mpiv_adj += pivot_.pivot_adj_inc_.value();
        ms_adj += wrist_.wrist_adj_inc_.value();
    } else if (pivot_down_manual) {
        mpiv_adj -= pivot_.pivot_adj_inc_.value();
        ms_adj -= wrist_.wrist_adj_inc_.value();
    }


    CoordinatePositions coordinateConverted = InverseKinematics::degree_toCoordinate({nextPivotTarget.to<double>() + mpiv_adj, 
        nextWristTarget.to<double>() + ms_adj, nextTelescopeTarget.to<double>() + mtele_adj});

    CoordinatePositions coordinateConvertedSecond = InverseKinematics::degree_toCoordinatePoint2({nextPivotTarget.to<double>() + mpiv_adj, 
        nextWristTarget.to<double>() + ms_adj, nextTelescopeTarget.to<double>() + mtele_adj});

    // std::cout << "FA" << coordinateConverted.forward_axis << std::endl;
    // std::cout << coordinateConverted.upward_axis << std::endl;

    if (!((InverseKinematics::withinBounds(coordinateConverted) 
        && InverseKinematics::withinBounds(coordinateConvertedSecond)))) {
        mpiv_adj = pmpiv_adj;
        ms_adj = pms_adj;
        mtele_adj = pmtele_adj;

        std::cout << "OUT of bounds, forward " << coordinateConverted.forward_axis << ", upward" << coordinateConverted.upward_axis << std::endl;
    } else {
        pmpiv_adj =  mpiv_adj;
        pms_adj = ms_adj;
        pmtele_adj = mtele_adj;
            
    }

    nextPivotTarget+=units::degree_t(mpiv_adj);
    nextTelescopeTarget+=units::inch_t(mtele_adj);
    nextWristTarget+=units::degree_t(ms_adj);

    telescope_.SetTarget({nextTelescopeTarget});
    pivot_.SetTarget({nextPivotTarget});
    wrist_.SetTarget({nextWristTarget});

    //TODO Adjustments

    //TODO: Intermediates?

    bool pivot_done_=units::math::abs(pivot_.readings().pivot_position-nextPivotTarget)<pivot_.pivot_position_tolerance_.value();
    bool telescope_done_=units::math::abs(telescope_.readings().extension-nextTelescopeTarget)<telescope_.telescope_position_tolerance_.value();
    bool wrist_done_=units::math::abs(wrist_.readings().wrist_position-nextWristTarget)<wrist_.wrist_position_tolerance_.value();

    is_done_ = pivot_done_&&telescope_done_&&wrist_done_;
  }
}

void OdometryShootCommand::End(bool interrupted) {
  Log("OdometryShoot Command Finished");
}

bool OdometryShootCommand::IsFinished() {
  return false;
}