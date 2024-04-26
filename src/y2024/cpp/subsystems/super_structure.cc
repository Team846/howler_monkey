#include "subsystems/super_structure.h"

#include <frc/DriverStation.h>

#include "frc846/util/share_tables.h"

SuperStructureSubsystem::SuperStructureSubsystem(bool init)
    : frc846::Subsystem<SuperStructureReadings, SuperStructureTarget>(
          "super_structure", init) {
  if (init) {
  }
}
SuperStructureTarget SuperStructureSubsystem::ZeroTarget() const {
  SuperStructureTarget target;
  return target;
}

bool SuperStructureSubsystem::VerifyHardware() { return true; }

SuperStructureReadings SuperStructureSubsystem::GetNewReadings() {
  if (auto alliance = frc::DriverStation::GetAlliance()) {
    if (alliance.value() == frc::DriverStation::Alliance::kRed) {
      frc846::util::ShareTables::SetBoolean("is_red_side", true);
    }
    if (alliance.value() == frc::DriverStation::Alliance::kBlue) {
      frc846::util::ShareTables::SetBoolean("is_red_side", false);
    }
  } else {
    frc846::util::ShareTables::SetBoolean("is_red_side", false);
  }

  return {};
}

void SuperStructureSubsystem::DirectWrite(SuperStructureTarget target) {
  if (hasZeroed) {
    return;
  }
}
