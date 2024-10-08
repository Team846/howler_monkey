#include "frc846/swerve/waypt_traversal_calculator.h"

#include "gtest/gtest.h"

class WayptTraversalCalculatorTest : public ::testing::Test {
 protected:
  frc846::swerve::WTCConstants constants_;
  WayptTraversalCalculatorTest()
      : calculator_{},
        constants_{13.0_fps,     16.0_fps_sq,  16.0_fps_sq, 15.0_fps,
                   100.0_fps_sq, 100.0_fps_sq, 3.0_in,      20_ms} {
    calculator_.setConstants(constants_);
  }

  frc846::swerve::WayptTraversalCalculator calculator_;
};

TEST_F(WayptTraversalCalculatorTest, HasCrossedWaypt) {
  frc846::swerve::WTCInput input{{0.0_ft, 0.0_ft},
                                 {10.0_ft, 0.0_ft},
                                 {0.0_ft, 0.0_ft},
                                 0_deg,
                                 0_deg,
                                 0.0_fps,
                                 0.0_fps};

  EXPECT_FALSE(calculator_.HasCrossedWaypt(input));

  input.current_pos = {5.0_ft, 0.0_ft};

  EXPECT_FALSE(calculator_.HasCrossedWaypt(input));

  input.current_pos = {10.0_ft, 0.0_ft};

  EXPECT_TRUE(calculator_.HasCrossedWaypt(input));

  input.current_pos = {14.0_ft, 0.0_ft};

  EXPECT_TRUE(calculator_.HasCrossedWaypt(input));
}

TEST_F(WayptTraversalCalculatorTest, ReqsDecel) {
  frc846::swerve::WTCInput input{{0.0_ft, 0.0_ft},
                                 {10.0_ft, 0.0_ft},
                                 {0.0_ft, 0.0_ft},
                                 0_deg,
                                 0_deg,
                                 10.0_fps,
                                 0.0_fps};

  frc846::swerve::WTCOutput output = calculator_.calculate(input);

  EXPECT_TRUE(output.target_vel.magnitude() > 10.0_fps);

  input.current_pos = {9.0_ft, 0.0_ft};

  output = calculator_.calculate(input);

  EXPECT_TRUE(output.target_vel.magnitude() < 10.0_fps);
}

TEST_F(WayptTraversalCalculatorTest, ReqsAccel) {
  frc846::swerve::WTCInput input{{0.0_ft, 0.0_ft},
                                 {10.0_ft, 0.0_ft},
                                 {0.0_ft, 0.0_ft},
                                 0_deg,
                                 0_deg,
                                 2.0_fps,
                                 0.0_fps};

  frc846::swerve::WTCOutput output = calculator_.calculate(input);

  EXPECT_TRUE(output.target_vel.magnitude() > 2.0_fps);

  input.current_pos = {5.0_ft, 0.0_ft};

  output = calculator_.calculate(input);

  EXPECT_TRUE(output.target_vel.magnitude() > 2.0_fps);

  input.current_vel = 4.0_fps;
  input.current_pos = {9.5_ft, 0.0_ft};

  output = calculator_.calculate(input);

  EXPECT_FALSE(output.target_vel.magnitude() > 2.0_fps);
}

TEST_F(WayptTraversalCalculatorTest, FullPathFakeTest) {
  frc846::swerve::WTCInput input{{0.0_ft, 0.0_ft},
                                 {10.0_ft, 0.0_ft},
                                 {0.0_ft, 0.0_ft},
                                 0_deg,
                                 0_deg,
                                 0.0_fps,
                                 0.0_fps};

  std::cout << "\n-----------------------------------" << std::endl;
  std::cout << "Starting test [FullPathFakeTest]\n" << std::endl;

  auto loop_time = 0.02_s;

  auto total_time = 0.0_s;

  for (int i{0}; i <= 100; i++) {
    auto output = calculator_.calculate(input);

    double current_dc = input.current_vel / constants_.true_max_spd;
    double target_dc = output.target_vel.magnitude() / constants_.true_max_spd;

    double acc_dc = (target_dc - current_dc);
    auto acc = acc_dc *
               (acc_dc > 0 ? constants_.true_max_acc : constants_.true_max_dec);

    if (output.crossed_waypt) {
      std::cout << "\nReached waypoint\n" << std::endl;
      break;
    }

    auto delta_vec_unit = (input.target_pos - input.current_pos).unit();
    auto est_vel = (input.current_vel + acc * loop_time);

    frc846::math::VectorND<units::feet_per_second_t, 2> est_vel_vec{
        est_vel * delta_vec_unit[0].to<double>(),
        est_vel * delta_vec_unit[1].to<double>()};

    input.current_vel = est_vel_vec.magnitude();

    input.current_pos +=
        {est_vel_vec[0] * loop_time, est_vel_vec[1] * loop_time};

    total_time += loop_time;

    std::cout << "Time [" << total_time.to<double>() << "] - Position ["
              << input.current_pos[0].to<double>() << ", "
              << input.current_pos[1].to<double>() << "]";
    std::cout << " - Velocity [" << est_vel_vec[0].to<double>() << ", "
              << est_vel_vec[1].to<double>() << "]" << std::endl;
  }

  std::cout << "End of test [FullPathFakeTest]" << std::endl;
  std::cout << "-----------------------------------\n" << std::endl;

  EXPECT_TRUE(true);
}