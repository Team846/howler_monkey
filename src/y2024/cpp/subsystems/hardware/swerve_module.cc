#include "subsystems/hardware/swerve_module.h"

#include <thread>

static constexpr auto kMotorType = rev::CANSparkMax::MotorType::kBrushless;

SwerveModuleSubsystem::SwerveModuleSubsystem(
    const frc846::Loggable& drivetrain, bool init, std::string location,
    units::degree_t fallback_cancoder_offset,
    frc846::control::ControlGainsHelper* drive_esc_gains_helper,
    frc846::control::ControlGainsHelper* steer_esc_gains_helper,
    units::foot_t drive_conversion, units::degree_t steer_conversion,
    int drive_esc_id, int steer_esc_id, int cancoder_id,
    frc846::Pref<units::ampere_t>& current_limit,
    frc846::Pref<units::ampere_t>& motor_stall_current,
    frc846::Pref<double>& braking_constant,
    frc846::Pref<units::feet_per_second_t>& max_speed)
    : frc846::Subsystem<SwerveModuleReadings, SwerveModuleTarget>{drivetrain,
                                                                  "module_" +
                                                                      location,
                                                                  init},
      cancoder_offset_{*this, "cancoder_offset", fallback_cancoder_offset},
      drive_esc_helper_{*this, location + "D", drive_esc_id},
      steer_esc_helper_{*this, location + "S", steer_esc_id},
      cancoder_{cancoder_id},
      current_limit_{current_limit},
      motor_stall_current_{motor_stall_current},
      braking_constant_{braking_constant},
      max_speed_{max_speed} {
  drive_esc_helper_.Setup(drive_esc_gains_helper, false,
                          frc846::control::kBrake);
  drive_esc_helper_.SetInverted(true);
  drive_esc_helper_.SetupConverter(drive_conversion);

  // Disable applied output frame
  // drive_esc_helper_.DisableStatusFrames(
  //     {rev::CANSparkLowLevel::PeriodicFrame::kStatus0});

  steer_esc_helper_.Setup(steer_esc_gains_helper, false,
                          frc846::control::kCoast);
  steer_esc_helper_.SetupConverter(steer_conversion);
  // steer_esc_helper_.DisableStatusFrames(
  //     {rev::CANSparkLowLevel::PeriodicFrame::kStatus0});

  // // Invert so that clockwise is positive when looking down on the robot
  // cancoder_.ConfigSensorDirection(true);

  current_speed_ = 0_fps;

  ZeroWithCANcoder();

  drive_esc_helper_.SetInverted(false);
}

void SwerveModuleSubsystem::ZeroCancoder() {
  std::shared_ptr<nt::NetworkTable> table_ =
      nt::NetworkTableInstance::GetDefault().GetTable(name());
  table_->SetDefaultNumber(
      "cancoder_offset",
      cancoder_.GetAbsolutePosition().GetValue().to<double>() * 360);
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
    module_direction = (-position.GetValue());

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

  zero_offset_ = module_direction - cancoder_offset_.value() -
                 steer_esc_helper_.GetPosition();

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
  FRC846_VERIFY(drive_esc_helper_.GetInverted() == false, ok,
                "drive esc incorrect invert state");
  FRC846_VERIFY(steer_esc_helper_.VerifyConnected(), ok,
                "steer esc not connected");
  FRC846_VERIFY(steer_esc_helper_.GetInverted() == false, ok,
                "steer esc incorrect invert state");
  return ok;
}

SwerveModuleReadings SwerveModuleSubsystem::GetNewReadings() {
  SwerveModuleReadings readings;

  readings.speed = drive_esc_helper_.GetVelocity();
  readings.direction = steer_esc_helper_.GetPosition() + zero_offset_;
  readings.distance = drive_esc_helper_.GetPosition();

  current_speed_ = readings.speed;

  current_graph_.Graph(units::ampere_t(drive_esc_helper_.GetCurrent()));

  swerve_speed_graph_.Graph((units::unit_cast<double>(readings.speed)));

  return readings;
}

void SwerveModuleSubsystem::DirectWrite(SwerveModuleTarget target) {
  target_speed_graph_.Graph(target.speed);
  target_direction_graph_.Graph(target.direction);
  direction_graph_.Graph(steer_esc_helper_.GetPosition());
  cancoder_graph_.Graph(-cancoder_.GetAbsolutePosition().GetValue());

  auto [normalized_angle, reverse_drive] =
      NormalizedDirection(readings().direction, target.direction);

  if (target.speed == 0_fps) {
    normalized_angle = last_direction_;
  } else {
    last_direction_ = normalized_angle;
  }

  units::feet_per_second_t target_velocity =
      target.speed * (reverse_drive ? -1 : 1);

  double drive_output;
  if (target.control == kClosedLoop) {
    drive_esc_helper_.Write(frc846::control::ControlMode::Velocity,
                            target_velocity);
  } else if (target.control == kOpenLoop) {
    drive_output = target_velocity / max_speed_.value();

    double braking_force = (target_velocity - readings().speed) /
                           max_speed_.value() * braking_constant_.value();

    drive_output += braking_force;

    double generatedEMF = 12.0 * readings().speed / max_speed_.value();

    double upperTargetEMF =
        12.0 * current_limit_.value() / motor_stall_current_.value() +
        generatedEMF;
    double lowerTargetEMF =
        12.0 * current_limit_.value() / motor_stall_current_.value() +
        generatedEMF;

    double upperDCBound = upperTargetEMF / 12.0;
    double lowerDCBound = lowerTargetEMF / 12.0;

    upper_dc_current_limiting_.Graph(upperDCBound);
    lower_dc_current_limiting_.Graph(lowerDCBound);

    drive_output = std::min(upperDCBound, std::max(lowerDCBound, drive_output));

    drive_esc_helper_.Write(frc846::control::ControlMode::Percent,
                            drive_output);
  } else {
    Warn("No Control Strategy Set");
  }

  units::degree_t steer_output = (normalized_angle - zero_offset_);
  steer_esc_helper_.Write(frc846::control::ControlMode::Position, steer_output);
}