#include "constants.h"

CoordinatePositions ArmKinematics::calculateCoordinatePosition(
    RawPositions pos_raw, bool intakePoint) {
  Vector2D<units::inch_t> ground_to_pivot_vector{pivotToCenter, pivotToGround};

  Vector2D<units::inch_t> pivot_first_segment_vector{0.0_in,
                                                     pivotToWristOffset};

  Vector2D<units::inch_t> arm_length_vector{pivotToWrist + pos_raw.extension,
                                            0.0_in};

  // Rotate is CW
  auto pivot_first_segment_vector_rotated =
      pivot_first_segment_vector.Rotate(-pos_raw.pivot_angle);

  auto arm_length_vector_rotated =
      arm_length_vector.Rotate(-pos_raw.pivot_angle);

  Vector2D<units::inch_t> joint_to_endpoint_vector{
      intakePoint ? shooterToIntake : 0.0_in, wristToShooter};

  auto joint_to_endpoint_vector_rotated = joint_to_endpoint_vector.Rotate(
      -pos_raw.pivot_angle +
      pos_raw.wrist_angle);  // Wrist is measured off of pivot, in opposite
                             // direction

  units::degree_t shootingAngle = pos_raw.wrist_angle - pos_raw.pivot_angle;

  Vector2D<units::inch_t> endpoint =
      ground_to_pivot_vector + pivot_first_segment_vector_rotated +
      arm_length_vector_rotated + joint_to_endpoint_vector_rotated;

  return {shootingAngle, endpoint.x, endpoint.y};
}