#include "subsystems/hardware/telescope.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

TelescopeSubsystem::TelescopeSubsystem(bool init)
    : frc846::Subsystem<TelescopeReadings, TelescopeTarget>{"telescope", init} {
  if (init) {
    telescope_esc_.Setup(&tele_esc_gains_, false);

    telescope_esc_.SetupConverter(0.25_in);

    telescope_esc_.ZeroEncoder();

    telescope_esc_.ConfigurePositionLimits(6_in, 0_in);

    telescope_esc_.DisableStatusFrames(
        {rev::CANSparkBase::PeriodicFrame::kStatus0,
         rev::CANSparkBase::PeriodicFrame::kStatus4,
         rev::CANSparkBase::PeriodicFrame::kStatus3});
  }
}

TelescopeTarget TelescopeSubsystem::ZeroTarget() const {
  TelescopeTarget target;
  target.extension = 0.0;
  return target;
}

bool TelescopeSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;

    FRC846_VERIFY(telescope_esc_.VerifyConnected(), ok,
                  "Telescope ESC not connected");
    FRC846_VERIFY(!telescope_esc_.GetInverted(), ok,
                  "Telescope ESC incorrect inversion");

    return ok;
  }
  return true;
}

TelescopeReadings TelescopeSubsystem::GetNewReadings() {
  TelescopeReadings readings;

  readings.extension = telescope_esc_.GetPosition();

  frc846::util::ShareTables::SetDouble("telescope_extension",
                                       readings.extension.to<double>());

  tele_pos_graph.Graph(readings.extension);

  if (auto target_ext = std::get_if<units::inch_t>(&target_.extension)) {
    tele_error_graph.Graph(*target_ext - readings.extension);
  }

  return readings;
}

void TelescopeSubsystem::PositionTelescope(TelescopeTarget target) {
  if (auto pos = std::get_if<units::inch_t>(&target.extension)) {
    telescope_esc_.Write(frc846::control::ControlMode::Position, *pos);

    target_tele_pos_graph.Graph(*pos);
  } else if (auto output = std::get_if<double>(&target.extension)) {
    telescope_esc_.Write(frc846::control::ControlMode::Percent, *output);

    target_tele_duty_cycle_graph.Graph(*output);
  }
}

void TelescopeSubsystem::DirectWrite(TelescopeTarget target) {
  PositionTelescope(target);
}