#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/velocity.h>

#include <map>

#include "frc846/math/calculator.h"
#include "frc846/math/vectors.h"
#include "frc846/ntinf/pref.h"

struct ATCalculatorInput {
  units::degree_t bearing;
  std::vector<units::degree_t> thetas;
  std::vector<units::inch_t> distances;
  std::vector<double> tags;
};

struct ATCalculatorOutput {
  frc846::math::VectorND<units::foot_t, 2> pos;
};

struct AprilTagData {
  units::inch_t x_pos;
  units::inch_t y_pos;
  units::degree_t angle;
  units::inch_t height;
};

struct ATCalculatorConstants {
  std::map<int, AprilTagData> tag_locations;
  units::inch_t camera_x_offset;
  units::inch_t camera_y_offset;
  units::degree_t cam_angle_offset;
};

class AprilTagCalculator
    : public frc846::math::Calculator<ATCalculatorInput, ATCalculatorOutput,
                                      ATCalculatorConstants> {
 public:
  AprilTagCalculator() {};

  ATCalculatorOutput calculate(ATCalculatorInput input) override;
  frc846::math::VectorND<units::foot_t, 2> getPos(units::degree_t bearing,
                                                  units::degree_t theta,
                                                  units::inch_t distance,
                                                  int tag);
};