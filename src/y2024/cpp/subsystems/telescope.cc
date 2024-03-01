#include "subsystems/telescope.h"
#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

TelescopeSubsystem::TelescopeSubsystem(bool init)
    : frc846::Subsystem<TelescopeReadings, TelescopeTarget>{"telescope", init} {
    if (init) {
        tele_one_.Setup(&tele_esc_gains_, false);

        tele_one_.SetupConverter(0.25_in);
        
        tele_one_.ZeroEncoder();
       
        tele_one_.ConfigurePositionLimits(8.5_in, 0_in);

        tele_one_.DisableStatusFrames({rev::CANSparkBase::PeriodicFrame::kStatus0, 
          rev::CANSparkBase::PeriodicFrame::kStatus4, 
          rev::CANSparkBase::PeriodicFrame::kStatus3});
    }
}

TelescopeTarget TelescopeSubsystem::ZeroTarget() const {
  TelescopeTarget target;
  target.extension = 0.0;
  return target;
}

TelescopeTarget TelescopeSubsystem::MakeTarget(std::variant<units::inch_t, double> tele_out) {
      TelescopeTarget target;
      target.extension = tele_out;
      return target;
}

bool TelescopeSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;

    FRC846_VERIFY(tele_one_.VerifyConnected(), ok, "Telescope ESC not connected");
    FRC846_VERIFY(!tele_one_.GetInverted(), ok, "Telescope ESC incorrect inversion");

    return ok;
  }
  return true;
}

TelescopeReadings TelescopeSubsystem::GetNewReadings() {
  TelescopeReadings readings;

  readings.extension = tele_one_.GetPosition();

  frc846::util::ShareTables::SetDouble("telescope_extension", readings.extension.to<double>());

  tele_pos_graph.Graph(readings.extension);

  return readings;
}

void TelescopeSubsystem::PositionTelescope(TelescopeTarget target) {
  if (auto pos = std::get_if<units::inch_t>(&target.extension)) {
    tele_one_.Write(frc846::control::ControlMode::Position, *pos);

    target_tele_pos_graph.Graph(*pos);
  } else if (auto output = std::get_if<double>(&target.extension)) {
    tele_one_.Write(frc846::control::ControlMode::Percent, *output);

    target_tele_duty_cycle_graph.Graph(*output);
  }
}

void TelescopeSubsystem::DirectWrite(TelescopeTarget target) {
  PositionTelescope(target);
}