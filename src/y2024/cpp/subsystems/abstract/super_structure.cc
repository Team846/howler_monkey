#include "subsystems/abstract/super_structure.h"

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

SuperStructureReadings SuperStructureSubsystem::GetNewReadings() { return {}; }

void SuperStructureSubsystem::DirectWrite(SuperStructureTarget target) {
  if (hasZeroed) {
    return;
  }
}
