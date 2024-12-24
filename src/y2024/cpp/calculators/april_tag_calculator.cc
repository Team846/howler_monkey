#include "calculators/april_tag_calculator.h"

ATCalculatorOutput AprilTagCalculator::calculate(ATCalculatorInput input) {
  if ((input.distances.size() == input.thetas.size() &&
       input.thetas.size() == input.tags.size())) {
    frc846::math::VectorND<units::foot_t, 2> pos;
    int succesfulTags = 0;
    for (int i = 0; i < input.distances.size(); i++) {
      if (constants_.tag_locations.contains(input.tags[i])) {
        pos += getPos(input.bearing, input.thetas[i], input.distances[i],
                      input.tags[i]);
        succesfulTags++;
      }
    }
    pos /= succesfulTags;
    if (succesfulTags != 0) {
      return {pos};
    }
  }

  return {{-1_ft, -1_ft}};
}

frc846::math::VectorND<units::foot_t, 2> AprilTagCalculator::getPos(
    units::degree_t bearing, units::degree_t theta, units::inch_t distance,
    int tag) {
  frc846::math::VectorND<units::foot_t, 2> local_tag_pos{
      -distance * units::math::sin(theta + constants_.cam_angle_offset) -
          constants_.camera_x_offset,
      distance * units::math::cos(theta + constants_.cam_angle_offset) +
          constants_.camera_y_offset};
  local_tag_pos = local_tag_pos.rotate(bearing);
  std::cout << local_tag_pos[0].to<double>() << std::endl;
  return {
      constants_.tag_locations[tag].x_pos - local_tag_pos[0],
      constants_.tag_locations[tag].y_pos - local_tag_pos[1],
  };
}