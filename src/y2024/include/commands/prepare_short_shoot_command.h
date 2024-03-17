#ifndef y2024_COMMANDS_PREPARE_SHOOT_COMMAND_H_
#define y2024_COMMANDS_PREPARE_SHOOT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/scorer.h"
#include "subsystems/pivot.h"
#include "subsystems/telescope.h"
#include "subsystems/wrist.h"
#include "subsystems/robot_container.h"

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

//     static constexpr double h = 81.0/12.0 - 23.0/12.0;
//     static constexpr double l = 14.0 / 12.0;

//     static constexpr double k = 0; //0.43;
//     static constexpr double w = 0; //105.0 * pi / 180.0;

//   public:

//   static double f_of_x(double v, double x, double d, double r_v, double r_o) {
//     double cosx = cos(x);
//     double sinx = sin(x);
//     return ((v*sinx)*(d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v)
//                 -1.0/2.0*g*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))*((d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/(v*cosx))*(r_o/(v*cosx)))+r_v))  
//                     +l*sinx/2+k*sin(x+w)-h);
//   }

//   static double f_prime(double v, double x, double d, double r_v, double r_o) {
//     double cosx = cos(x);
//     double sinx = sin(x);

//     double t = 0.0;
//     double t_prime = 0.0;

//     try {
//       t = (d-l*cosx/2)/(v*cosx*sqrt(1-(r_o/v*cosx))*(r_o/v*cosx)+r_v);
//       t_prime = (l*r_v*v*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v)) + l*r_o*r_o*cosx*cosx*cosx - 4*d*r_o*r_o*cosx*cosx + 2*d*v*v)*sinx /
//                       (2*v*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v))*(r_v+v*cosx*sqrt(1-(r_o*cosx/v)*(r_o*cosx/v))));
//     } catch (std::exception exc) {
//       std::cout << "Out of Range" << std::endl;
//     }

//     return v*cosx*t + v*sinx*t_prime + l*cosx/2 - 1.0/2.0*g*2*t*t_prime;
//   }

//   static double calculate(double v, double d, double r_v, double r_o, 
//     double initial_guess = radians(1.01), double tolerance=0.1, double max_iterations=20000) {
//     double x = initial_guess;
//     for (int i = 0; i < max_iterations; i++) {
//         auto fx = f_of_x(v, x, d, r_v, r_o);
//         if (std::abs(fx) < tolerance) {
//           return degs(x);
//         }

//         x -= std::min(radians(120.0/std::max(14, (i+1))), 
//           std::max(radians(-120.0/std::max(14, (i+1))), (fx / f_prime(v, x, d, r_v, r_o) / 1000.0)));

//         if (x < 0.0) x = radians(1.01);
//         else if (x > pi / 2) x = radians(1.01);
//     }
//     return degs(x);
//   }
// };

class PrepareShortShootCommand
    : public frc2::CommandHelper<frc2::Command, PrepareShortShootCommand>,
      public frc846::Loggable {
 public:
  PrepareShortShootCommand(RobotContainer& container, double shooting_distance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ScorerSubsystem& scorer_;
  PivotSubsystem& pivot_;
  TelescopeSubsystem& telescope_;
  WristSubsystem& wrist_;

  double dist_;

  bool is_done_ = false;
};

#endif  // y2024_COMMANDS_PREPARE_SHOOT_COMMAND_H_