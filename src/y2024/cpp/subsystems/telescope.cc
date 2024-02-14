#include "subsystems/telescope.h"
#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

TelescopeSubsystem::TelescopeSubsystem(bool init)
    : frc846::Subsystem<TelescopeReadings, TelescopeTarget>{"telescope", init} {
    if (init) {
        tele_one_.Setup(&tele_esc_gains_, true);
        tele_two_.Setup(&tele_esc_gains_, true);
        tele_one_.SetupConverter(2.2146_in);
        tele_two_.SetupConverter(2.2146_in);
        tele_one_.ZeroEncoder();
        tele_two_.ZeroEncoder();

        tele_one_.ConfigurePositionLimits(10_in, 0.5_in);
        tele_two_.ConfigurePositionLimits(10_in, 0.5_in);

        // tele_tandem_.Setup(&tele_esc_gains_);
        // tele_tandem_.SetInverted({true, true});
        // tele_tandem_.SetupConverter(2_in);
        // tele_tandem_.ZeroEncoder();

        // tele_tandem.Setup(&tele_esc_gains_);
        // tele_tandem.SetupConverter(1.0_in);
        // tele_tandem.ZeroEncoder();

        // tele_tandem.AddESC(tele_one_esc_.that());
        // tele_tandem.AddESC(tele_two_esc_.that());
        // tele_tandem.SetupAll(&tele_esc_gains_, false);
        // tele_tandem.SetupAllConversions(1_in);
        // tele_tandem.ZeroEncoders();
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

    FRC846_VERIFY(tele_one_.VerifyConnected(), ok, "Tele one not connected");
    FRC846_VERIFY(tele_two_.VerifyConnected(), ok, "Tele two not connected");

    return ok;
  }
  return true;
}

TelescopeReadings TelescopeSubsystem::GetNewReadings() {
  TelescopeReadings readings;

  readings.extension = tele_one_.GetPosition();

  frc846::util::ShareTables::SetVal("telescope_extension", readings.extension.to<double>());

  tele_pos_graph.Graph(readings.extension);

  return readings;
}

void TelescopeSubsystem::PositionTelescope(TelescopeTarget target) {
  if (auto pos = std::get_if<units::inch_t>(&target.extension)) {
    tele_one_.Write(frc846::control::ControlMode::Position, *pos);
    tele_two_.Write(frc846::control::ControlMode::Position, *pos);

    target_tele_pos_graph.Graph(*pos);
  } else if (auto output = std::get_if<double>(&target.extension)) {
    tele_one_.Write(frc846::control::ControlMode::Percent, *output);
    tele_two_.Write(frc846::control::ControlMode::Percent, *output);

    target_tele_duty_cycle_graph.Graph(*output);
  }
}

void TelescopeSubsystem::DirectWrite(TelescopeTarget target) {
  PositionTelescope(target);
}