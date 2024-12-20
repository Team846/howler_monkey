#include "subsystems/hardware/telescope.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

TelescopeSubsystem::TelescopeSubsystem(bool init)
    : frc846::robot::GenericSubsystem<TelescopeReadings, TelescopeTarget>{
          "telescope", init} {
  if (init) {
    telescope_esc_.Init(frc846::control::REVSparkType::kSparkMAX);
  }
}

void TelescopeSubsystem::Setup() {
  telescope_esc_.Configure({frc846::control::DataTag::kPositionData});

  telescope_esc_.ZeroEncoder(0.0_in);
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

TelescopeReadings TelescopeSubsystem::ReadFromHardware() {
  TelescopeReadings readings;

  readings.extension = telescope_esc_.GetPosition();

  frc846::util::ShareTables::SetDouble("telescope_extension",
                                       readings.extension.to<double>());

  tele_pos_graph.Graph(readings.extension);

  auto target_output = GetTarget().extension;
  if (auto target_ext = std::get_if<units::inch_t>(&target_output)) {
    tele_error_graph.Graph(*target_ext - readings.extension);
  }

  return readings;
}

void TelescopeSubsystem::WriteToHardware(TelescopeTarget target) {
  if (auto pos = std::get_if<units::inch_t>(&target.extension)) {
    telescope_esc_.WritePosition(*pos);

    target_tele_pos_graph.Graph(*pos);
  } else if (auto output = std::get_if<double>(&target.extension)) {
    telescope_esc_.WriteDC(*output);

    target_tele_duty_cycle_graph.Graph(*output);
  }
}