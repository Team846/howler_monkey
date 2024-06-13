#include "subsystems/abstract/super_structure.h"

#include "frc846/util/share_tables.h"

SuperStructureTarget SuperStructureSubsystem::ZeroTarget() const {
  SuperStructureTarget target;
  return target;
}

bool SuperStructureSubsystem::VerifyHardware() { return true; }

SuperStructureReadings SuperStructureSubsystem::GetNewReadings() { return {}; }

void SuperStructureSubsystem::DirectWrite(SuperStructureTarget target) {
  PTWSetpoint targetPos = currentSetpoint + manualAdjustments;

  pivot_->SetTarget({targetPos.pivot});
  telescope_->SetTarget({targetPos.telescope});
  wrist_->SetTarget({targetPos.wrist});
}
