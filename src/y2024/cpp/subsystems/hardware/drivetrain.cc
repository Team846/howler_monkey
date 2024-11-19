#include "subsystems/hardware/drivetrain.h"

#include <stdexcept>

#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"
#include "subsystems/hardware/swerve_module.h"

DrivetrainSubsystem::DrivetrainSubsystem(bool initialize)
    : frc846::robot::GenericSubsystem<DrivetrainReadings,
                                      DrivetrainTarget>{"drivetrain",
                                                        initialize},
      odometry_{frc846::math::VectorND<units::foot_t, 2>{0.0_ft, 0.0_ft}} {
  bearing_offset_ = 0_deg;
  ZeroOdometry();
  frc::SmartDashboard::PutData("Field", &m_field);
}

void DrivetrainSubsystem::ZeroModules() {
  if (!is_initialized()) return;

  Log("Zeroed modules");
  for (auto module : modules_all_) {
    module->ZeroWithCANcoder();
  }
}

void DrivetrainSubsystem::ZeroCancoders() {
  if (!is_initialized()) return;

  for (auto module : modules_all_) {
    Log("Zeroing Cancoder");
    module->ZeroCancoder();
  }
}

void DrivetrainSubsystem::ZeroBearing() {
  if (!is_initialized()) return;

  // Attempt to zero using the gyro, and retry if the gyro is disconnected or
  // calibrating

  constexpr int kMaxAttempts = 5;
  constexpr int kSleepTimeMs = 500;
  bearing_offset_ = 0_deg;
  for (int attempts = 1; attempts <= kMaxAttempts; ++attempts) {
    Log("Gyro zero attempt {}/{}", attempts, kMaxAttempts);
    if (gyro_.IsConnected() && !gyro_.IsCalibrating()) {
      gyro_.ZeroYaw();
      Log("Zeroed bearing");
      break;
    }

    Log("Unable to zero", attempts, kMaxAttempts);

    if (attempts == kMaxAttempts) {
      Error("NOT ZEROED!!!");
    } else {
      Log("Sleeping {}ms...", kSleepTimeMs);
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
    }
  }

  ZeroModules();
}

void DrivetrainSubsystem::ZeroOdometry() {
  if (!is_initialized()) return;

  bearing_offset_ = 0_deg;
  odometry_.Zero();
}

void DrivetrainSubsystem::SetPoint(
    frc846::math::VectorND<units::foot_t, 2> point) {
  if (!is_initialized()) return;

  Log("set point x {} y {}", point[0], point[1]);
  odometry_.SetPoint(point);
}

void DrivetrainSubsystem::SetBearing(units::degree_t bearing) {
  if (!is_initialized()) return;

  ZeroBearing();
  bearing_offset_ = bearing;
  Log("Zeroed bearing {}", bearing);
}

std::array<frc846::math::VectorND<units::feet_per_second_t, 2>,
           DrivetrainSubsystem::kModuleCount>
DrivetrainSubsystem::SwerveControl(
    frc846::math::VectorND<units::feet_per_second_t, 2> translation,
    units::degrees_per_second_t rotation_speed, units::inch_t width,
    units::inch_t height, units::inch_t radius,
    units::feet_per_second_t max_speed) {
  // Locations of each module
  static frc846::math::VectorND<units::dimensionless_t, 2>
      kModuleLocationSigns[DrivetrainSubsystem::kModuleCount] = {
          {-1, +1},  // fl
          {+1, +1},  // fr
          {-1, -1},  // bl
          {+1, -1},  // br
      };

  std::array<frc846::math::VectorND<units::feet_per_second_t, 2>,
             DrivetrainSubsystem::kModuleCount>
      module_targets;

  units::feet_per_second_t max_magnitude = 0_fps;
  for (int i = 0; i < kModuleCount; ++i) {
    // Location of the module relaive to the center
    frc846::math::VectorND<units::inch_t, 2> location{
        kModuleLocationSigns[i][0] * width / 2,
        kModuleLocationSigns[i][1] * height / 2,
    };

    // Target direction for the module - angle from center of robot to
    // module + 90 degrees
    //
    // x and y inputs in atan2 are swapped to return a bearing
    units::degree_t direction =
        units::math::atan2(location[0], location[1]) + 90_deg;

    // do 90 - direction to convert bearing to cartesian angle
    frc846::math::VectorND<units::feet_per_second_t, 2> rotation{
        rotation_speed * units::math::cos(90_deg - direction) * radius / 1_rad,
        rotation_speed * units::math::sin(90_deg - direction) * radius / 1_rad,
    };

    module_targets[i] = translation + rotation;
    if (module_targets[i].magnitude() <= max_speed * 4) {
      max_magnitude =
          units::math::max(max_magnitude, module_targets[i].magnitude());
    }
  }

  // Cap module speed if any of them exceed the max speed.
  // TODO unit test this.
  if (max_magnitude > max_speed) {
    auto scale = max_speed / max_magnitude;
    for (auto& t : module_targets) {
      t[0] *= scale;
      t[1] *= scale;
    }
  }

  return module_targets;
}

DrivetrainTarget DrivetrainSubsystem::ZeroTarget() const {
  DrivetrainTarget target;
  target.v_x = 0_fps;
  target.v_y = 0_fps;
  target.translation_reference = DrivetrainTranslationReference::kRobot;
  target.rotation = DrivetrainRotationVelocity(0_deg_per_s);
  target.control = kOpenLoop;

  return target;
}

bool DrivetrainSubsystem::VerifyHardware() {
  if (!is_initialized()) return true;

  bool ok = true;
  FRC846_VERIFY(gyro_.IsConnected(), ok, "gyro is not connected");
  FRC846_VERIFY(!gyro_.IsCalibrating(), ok, "gyro is calibrating");

  for (auto module : modules_all_) {
    bool module_ok = module->VerifyHardware();
    if (!module_ok) {
      module->Error("Failed hardware verification!!");
    }
    ok = ok && module_ok;
  }

  return ok;
}

DrivetrainReadings DrivetrainSubsystem::ReadFromHardware() {
  DrivetrainReadings readings{};

  readings.is_gyro_connected = gyro_.IsConnected();

  auto bearing = units::degree_t(gyro_.GetYaw()) + bearing_offset_;
  readings.angular_velocity = units::degrees_per_second_t(gyro_.GetRate());

  auto pitch_initial = units::degree_t(gyro_.GetPitch());
  auto roll_initial = units::degree_t(gyro_.GetRoll());
  readings.tilt = units::degree_t{pitch_initial * units::math::cos(bearing) +
                                  roll_initial * units::math::sin(bearing)};
  // auto current_time_ = frc846::wpilib::CurrentFPGATime();

  // Gets the position difference vector for each module, to update odometry
  // with
  std::array<frc846::math::VectorND<units::foot_t, 2>, kModuleCount>
      module_outs;
  for (int i = 0; i < kModuleCount; ++i) {
    modules_all_[i]->UpdateReadings();
    auto d = modules_all_[i]->GetReadings().distance;
    units::radian_t a = modules_all_[i]->GetReadings().direction;

    frc846::math::VectorND<units::foot_t, 2> vec{
        d * units::math::sin(a),
        d * units::math::cos(a),
    };
    module_outs[i] = vec;
  }

  odometry_.Update(module_outs, bearing);

  units::feet_per_second_t total_x = 0_fps, total_y = 0_fps;
  for (auto module : modules_all_) {
    total_x += module->GetReadings().speed *
               units::math::cos(90_deg - module->GetReadings().direction);
    total_y += module->GetReadings().speed *
               units::math::sin(90_deg - module->GetReadings().direction);
  }

  frc846::math::VectorND<units::feet_per_second_t, 2> unfiltered_velocity = {
      total_x / kModuleCount, total_y / kModuleCount};

  readings.pose = frc846::math::FieldPoint(odometry_.position(), bearing,
                                           unfiltered_velocity);

  pose_x_graph_.Graph(odometry_.position()[0]);
  pose_y_graph_.Graph(odometry_.position()[1]);
  // pose_bearing_graph.Graph(odometry_.position().bearing);
  v_x_graph_.Graph(readings.pose.velocity[0]);
  v_y_graph_.Graph(readings.pose.velocity[1]);

  frc846::util::ShareTables::SetDouble("robot_bearing_",
                                       readings.pose.bearing.to<double>());
  frc846::util::ShareTables::SetDouble("odometry_x_",
                                       odometry_.position()[0].to<double>());
  frc846::util::ShareTables::SetDouble("odometry_y_",
                                       odometry_.position()[1].to<double>());
  frc846::util::ShareTables::SetDouble("velocity_x_",
                                       readings.pose.velocity[0].to<double>());
  frc846::util::ShareTables::SetDouble("velocity_y_",
                                       readings.pose.velocity[1].to<double>());

  SetMap();

  vel_readings_composite_x =
      units::unit_cast<double>(readings.pose.velocity[0]);

  if (vel_readings_composite_x < 0.0) {
    vel_readings_composite_x = -vel_readings_composite_x;
  }

  vel_readings_composite_y =
      units::unit_cast<double>(readings.pose.velocity[1]);

  if (vel_readings_composite_y < 0.0) {
    vel_readings_composite_y = -vel_readings_composite_y;
  }

  vel_readings_composite = units::feet_per_second_t(
      sqrt(vel_readings_composite_x * vel_readings_composite_x +
           vel_readings_composite_y * vel_readings_composite_y));

  return readings;
}

void DrivetrainSubsystem::SetMap() {
  // set odometry
  m_field.SetRobotPose(frc::Pose2d(odometry_.position()[1],
                                   -odometry_.position()[0],
                                   GetReadings().pose.bearing));
}

void DrivetrainSubsystem::WriteToHardware(DrivetrainTarget target) {
  // Graph target
  target_v_x_graph_.Graph(target.v_x);
  target_v_y_graph_.Graph(target.v_y);
  target_translation_reference_graph_.Graph(
      target.translation_reference == DrivetrainTranslationReference::kField
          ? "field"
          : "robot");
  if (auto* target_rotation =
          std::get_if<DrivetrainRotationPosition>(&target.rotation)) {
    target_rotation_position_graph_.Graph(GetReadings().pose.bearing -
                                          *target_rotation);
    target_rotation_velocity_graph_.Graph(0_deg_per_s);
    // bearing_error.Graph(*target_rotation-GetReadings().pose.bearing);
  } else if (auto* target_rotation =
                 std::get_if<DrivetrainRotationVelocity>(&target.rotation)) {
    target_rotation_position_graph_.Graph(0_deg);
    target_rotation_velocity_graph_.Graph(*target_rotation);
  } else {
    throw std::runtime_error{"unhandled case"};
  }

  frc846::math::VectorND<units::feet_per_second_t, 2> target_translation = {
      target.v_x, target.v_y};
  // rotate translation vector if field oriented
  if (target.translation_reference == DrivetrainTranslationReference::kField) {
    units::degree_t offset = GetReadings().pose.bearing;
    target_translation = target_translation.rotate(-offset);
  }

  target_translation[0] = vx_ramp_rate_.Calculate(target_translation[0]);
  target_translation[1] = vy_ramp_rate_.Calculate(target_translation[1]);

  units::degrees_per_second_t target_omega;
  if (auto* theta = std::get_if<DrivetrainRotationPosition>(&target.rotation)) {
    // position control
    auto p_error =
        frc846::math::CoterminalDifference(*theta, GetReadings().pose.bearing);
    auto d_error = GetReadings().angular_velocity;

    target_omega = units::degrees_per_second_t(
        bearing_gains_p_.value() * p_error.to<double>() +
        bearing_gains_d_.value() * d_error.to<double>());
  } else if (auto* omega =
                 std::get_if<DrivetrainRotationVelocity>(&target.rotation)) {
    // velocity control
    target_omega = *omega;
  } else {
    throw std::runtime_error{"unhandled case"};
  }

  // Max speed in open loop vs closed loop
  auto max_speed = target.control == kOpenLoop ? max_speed_.value()
                                               : auto_max_speed_.value();

  auto targets = SwerveControl(target_translation, target_omega, width_.value(),
                               height_.value(), module_radius_, max_speed);

  frc::SmartDashboard::PutNumber(
      "velocity_error", (sqrt(units::unit_cast<double>(target.v_x) *
                                  units::unit_cast<double>(target.v_x) +
                              units::unit_cast<double>(target.v_y) *
                                  units::unit_cast<double>(target.v_y))) -
                            units::unit_cast<double>(vel_readings_composite));
  frc846::util::ShareTables::SetDouble(
      "velocity", (units::unit_cast<double>(vel_readings_composite)));

  for (int i = 0; i < kModuleCount; ++i) {
    modules_all_[i]->SetTarget(
        {targets[i].magnitude(), targets[i].angle(true), target.control});
    modules_all_[i]->UpdateHardware();
  }
}