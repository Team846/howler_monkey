#include "subsystems/super_structure.h"
#include "frc846/util/share_tables.h"

SuperStructureSubsystem::SuperStructureSubsystem(bool init)
    : frc846::Subsystem<SuperStructureReadings, SuperStructureTarget>("super_structure", init) {
        if (init){
            
        }
}
SuperStructureTarget SuperStructureSubsystem::ZeroTarget() const {
  SuperStructureTarget target;
  return target;
}

bool SuperStructureSubsystem::VerifyHardware() { return true; }

SuperStructureReadings SuperStructureSubsystem::GetNewReadings() { 
  frc846::util::ShareTables::SetBoolean("is_red_side", is_red_side_.value());
  return {}; 
}

void SuperStructureSubsystem::DirectWrite(SuperStructureTarget target) {
  if (hasZeroed) {
    return;
  }
}
