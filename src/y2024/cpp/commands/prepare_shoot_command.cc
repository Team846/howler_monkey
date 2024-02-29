// #include "commands/prepare_shoot_command.h"

// #include <frc/RobotBase.h>

// #include <cmath>

// #include "frc846/util/math.h"
// #include "frc846/wpilib/time.h"

// class ShootingCalculator {
//   private:
//     static double pow(double base, int exponent) {
//       for (int i = 0; i < exponent; i++) {
//         base *= base;
//       }
//       return base;
//     }

//     static constexpr double pi = 3.14159265;

//     static double radians(double degs) {
//       return degs * pi / 180;
//     }

//     static double degs(double radians) {
//       return radians * 180 / pi;
//     }

//     static constexpr double g = 32.0;

//     static constexpr double h = 78/12 - 26/12;
//     static constexpr double l = 14.0 / 12.0;

//     static constexpr double v = 25.5;

//     static constexpr double k = 0; //0.43;
//     static constexpr double w = 0; //105.0 * pi / 180.0;

//   public:

//   static double f_of_x(double x, double d, double r_v, double r_o) {
//     double cosx = cos(x);
//     double sinx = sin(x);
//     return ((v*sinx)*(d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v)
//                 -1.0/2.0*g*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))  
//                     +l*sinx/2+k*sin(x+w)-h);
//   }

//   static double f_prime(double x, double d, double r_v, double r_o) {
//     double cosx = cos(x);
//     double sinx = sin(x);

//     double t = 0.0;
//     double t_prime = 0.0;

//     try {
//       t = (d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/v*cosx))*(r_o/v*cosx)+r_v);
//       t_prime = (l*r_v*v*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v)) + l*r_o*r_o*cosx*cosx*cosx - 4*d*r_o*r_o*cosx*cosx + 2*d*v*v)*sinx /
//                       (2*v*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v))*(r_v+v*cosx*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v))));
//     } catch (std::exception exc) {}

//     return v*cosx*t + v*sinx*t_prime + l*cosx/2 - 1.0/2.0*g*2*t*t_prime;
//   }

//   static double calculate(double d, double r_v = 0.0, double r_o = 0.0, 
//     double initial_guess = radians(30), double tolerance=0.01, double max_iterations=150) {
//     double x = initial_guess;
//     for (int i = 0; i < max_iterations; i++) {
//         auto fx = f_of_x(x, d, r_v, r_o);
//         if (std::abs(fx) < tolerance) {
//           return degs(x);
//         }

//         x -= std::min(radians(120.0/std::max(10, (i+1))), 
//           std::max(radians(-120.0/std::max(10, (i+1))), 100 * fx / f_prime(x, d, r_v, r_o)));

//         if (x < 0.0) x = radians(45);
//         else if (x > pi / 2) x = radians(45);
//     }

//     return 0.0;
//   }
// };

// PrepareShootCommand::PrepareShootCommand(
//     RobotContainer& container, units::foot_t shooting_distance)
//     : frc846::Loggable{"prepare_shoot_command"},
//       scorer_(container.scorer_), pivot_(container.pivot_), 
//         telescope_(container.telescope_), wrist_(container.wrist_), 
//           dist_(shooting_distance.to<double>()) {
//   AddRequirements({&pivot_, &telescope_, &wrist_});
//   SetName("prepare_shoot_command");
// }

// void PrepareShootCommand::Initialize() {
//   Log("Prepare Shoot Command Initialize");
// }

// void PrepareShootCommand::Execute() {
//   scorer_.SetTarget(scorer_.MakeTarget(kSpinUp));

//   pivot_.SetTarget(pivot_.MakeTarget(37_deg));
//   telescope_.SetTarget(telescope_.MakeTarget(0_in));

//   units::degree_t theta = units::degree_t(ShootingCalculator::calculate(dist_, 
//     0.0, 0.0));

//   wrist_.SetTarget(wrist_.MakeTarget(110_deg - wrist_.wrist_home_offset_.value() + theta));

//   is_done_ = true;
// }

// void PrepareShootCommand::End(bool interrupted) {
//   Log("Prepare Shoot Command Finished");
// }

// bool PrepareShootCommand::IsFinished() {
//   return is_done_;
// }