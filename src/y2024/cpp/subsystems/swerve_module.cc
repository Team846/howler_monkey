#include "subsystems/swerve_module.h"

#include <thread>

#include "frc846/motor/config.h"

static constexpr auto kMotorType = rev::CANSparkMax::MotorType::kBrushless;

SwerveModuleSubsystem::SwerveModuleSubsystem(
    const frc846::Loggable& drivetrain, bool init, std::string location,
    units::degree_t fallback_cancoder_offset,
    frc846::motor::SparkMAXConfigHelper* drive_esc_config_helper,
    frc846::motor::GainsHelper* drive_esc_gains_helper,
    frc846::motor::SparkMAXConfigHelper* steer_esc_config_helper,
    frc846::motor::GainsHelper* steer_esc_gains_helper,
    frc846::Converter<units::foot_t>& drive_converter,
    frc846::Converter<units::degree_t>& steer_converter,
    int drive_esc_id, int steer_esc_id, int cancoder_id
  ) : frc846::Subsystem<SwerveModuleReadings, SwerveModuleTarget>{
          drivetrain,
          "module_" + location,
          init
      },
      cancoder_offset_{*this, "cancoder_offset", fallback_cancoder_offset},
      
      drive_converter_(drive_converter),
      steer_converter_(steer_converter),
      drive_esc_{drive_esc_id, kMotorType},
      steer_esc_{steer_esc_id, kMotorType},
      drive_esc_helper_{*this, drive_esc_, drive_esc_config_helper, drive_esc_gains_helper},
      steer_esc_helper_{*this, steer_esc_, steer_esc_config_helper, steer_esc_gains_helper},
      cancoder_{cancoder_id} {
  drive_esc_helper_.OnInit([&] {
    drive_esc_.SetInverted(true);

    // Disable applied output frame
    drive_esc_helper_.DisableStatusFrames(
        {rev::CANSparkLowLevel::PeriodicFrame::kStatus0});
  });

  steer_esc_helper_.OnInit([&] {
    // Disable applied output frame
    steer_esc_helper_.DisableStatusFrames(
        {rev::CANSparkLowLevel::PeriodicFrame::kStatus0});
  });

  drive_esc_helper_.Setup();
  steer_esc_helper_.Setup();

  // // Invert so that clockwise is positive when looking down on the robot
  // cancoder_.ConfigSensorDirection(true);

  current_speed_ = 0_fps;

  ZeroWithCANcoder();
}

void SwerveModuleSubsystem::ZeroCancoder() {
  std::shared_ptr<nt::NetworkTable> table_ =
      nt::NetworkTableInstance::GetDefault().GetTable(name());
  table_->SetDefaultNumber("cancoder_offset", cancoder_.GetAbsolutePosition().GetValue().to<double>() * 360);
  ZeroWithCANcoder();
}

std::pair<units::degree_t, bool> SwerveModuleSubsystem::NormalizedDirection(
    units::degree_t current, units::degree_t target) {
  units::degree_t normalized_direction =
      current + (-units::math::fmod(current, 360_deg) + target);

  units::degree_t abs_err = units::math::abs(normalized_direction - current);

  if (abs_err > units::math::abs(normalized_direction - 360_deg - current)) {
    normalized_direction -= 360_deg;
  } else if (abs_err >
             units::math::abs(normalized_direction + 360_deg - current)) {
    normalized_direction += 360_deg;
  }

  bool reverse_drive = false;
  if ((normalized_direction - current) > 90_deg) {
    normalized_direction = -(180_deg - normalized_direction);
    reverse_drive = true;
  } else if ((normalized_direction - current) < -90_deg) {
    normalized_direction = normalized_direction + 180_deg;
    reverse_drive = true;
  }
  return {normalized_direction, reverse_drive};
}

void SwerveModuleSubsystem::ZeroWithCANcoder() {
  constexpr int kMaxAttempts = 5;
  constexpr int kSleepTimeMs = 500;

  units::degree_t module_direction = 0_deg;
  for (int attempts = 1; attempts <= kMaxAttempts; ++attempts) {
    Log("CANCoder zero attempt {}/{}", attempts, kMaxAttempts);
    auto position = cancoder_.GetAbsolutePosition();
    module_direction = units::degree_t(position.GetValue());

    if (position.IsAllGood()) {
      Log("Zeroed to {}!", module_direction);
      break;
    }

    Warn("Unable to zero", attempts, kMaxAttempts);

    if (attempts == kMaxAttempts) {
      Error("NOT ZEROED!!!");
    } else {
      Log("Sleeping {}ms...", kSleepTimeMs);
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
    }
  }

  zero_offset_ =
      module_direction - cancoder_offset_.value() -
      steer_converter_.NativeToRealPosition(
          steer_esc_helper_.encoder().GetPosition());

  last_direction_ = module_direction;
}

SwerveModuleTarget SwerveModuleSubsystem::ZeroTarget() const {
  SwerveModuleTarget target;
  target.speed = 0_fps;
  target.direction = 0_deg;
  return target;
}

bool SwerveModuleSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(drive_esc_helper_.VerifyConnected(), ok,
                "drive esc not connected");
  FRC846_VERIFY(drive_esc_.GetInverted() == true, ok,
                "drive esc incorrect invert state");
  FRC846_VERIFY(steer_esc_helper_.VerifyConnected(), ok,
                "steer esc not connected");
  FRC846_VERIFY(steer_esc_.GetInverted() == false, ok,
                "steer esc incorrect invert state");
  return ok;
}

SwerveModuleReadings SwerveModuleSubsystem::GetNewReadings() {
  SwerveModuleReadings readings;

  readings.speed = drive_converter_.NativeToRealVelocity(
    drive_esc_helper_.encoder().GetVelocity());
  readings.direction =
      steer_converter_.NativeToRealPosition(
          steer_esc_helper_.encoder().GetPosition()) +
      zero_offset_;
  readings.distance = drive_converter_.NativeToRealPosition(
      drive_esc_helper_.encoder().GetPosition());

  current_speed_ = readings.speed;

  current_graph_.Graph(units::ampere_t(drive_esc_.GetOutputCurrent()));

  swerve_speed_graph_.Graph((units::unit_cast<double>(readings.speed)));

  return readings;
}

void SwerveModuleSubsystem::DirectWrite(SwerveModuleTarget target) {
  target_speed_graph_.Graph(target.speed);
  target_direction_graph_.Graph(target.direction);
  direction_graph_.Graph(steer_converter_.NativeToRealPosition(
      steer_esc_helper_.encoder().GetPosition()));
  cancoder_graph_.Graph(cancoder_.GetAbsolutePosition().GetValue());

  auto [normalized_angle, reverse_drive] =
      NormalizedDirection(readings().direction, target.direction);

  if (target.speed == 0_fps) {
    normalized_angle = last_direction_;
  } else {
    last_direction_ = normalized_angle;
  }

  units::feet_per_second_t target_velocity =
      target.speed * (reverse_drive ? -1 : 1);

  auto speed_difference = -target_velocity+current_speed_;

  double drive_output;
  if (target.control == kClosedLoop) {
    drive_output = drive_converter_.RealToNativeVelocity(target_velocity);
    drive_esc_helper_.Write(
        {frc846::motor::ControlMode::Velocity, drive_output});
  } else if (target.control == kOpenLoop) {
    if (units::unit_cast<double>(units::math::abs(speed_difference)) > units::unit_cast<double>(kControlChangeThresh)) {
      units::feet_per_second_t adjusted_target_velocity = units::feet_per_second_t(units::unit_cast<double>(current_speed_) - (units::unit_cast<double>(speed_difference) * kSpeedAdjustingFactor));
      drive_output = adjusted_target_velocity / kMaxSpeed;

      swerve_target_graph_.Graph(std::abs(units::unit_cast<double>(adjusted_target_velocity)));

      if (drive_output > 1.0) {
        drive_output = 1.0;
      } else if (drive_output < -1.0) {
        drive_output = -1.0;
      }

      drive_esc_helper_.Write(
          {frc846::motor::ControlMode::Percent, drive_output});
    } else {
      drive_output = target_velocity / kMaxSpeed;

      drive_esc_helper_.Write(
          {frc846::motor::ControlMode::Percent, drive_output});
    }
  } else {
    Warn("No Control Strategy Set");
  }

  double steer_output =
      steer_converter_.RealToNativePosition(normalized_angle - zero_offset_);
  steer_esc_helper_.Write({frc846::motor::ControlMode::Position, steer_output});
}