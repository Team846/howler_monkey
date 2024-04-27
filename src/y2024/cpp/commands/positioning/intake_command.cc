#include "commands/positioning/intake_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

#include "subsystems/setpoints.h"

IntakeCommand::IntakeCommand(
    RobotContainer& container)
    : frc846::Loggable{"intake_command"}, pivot_(container.pivot_),
        telescope_(container.telescope_), wrist_(container.wrist_), operator_(container.operator_) {
  AddRequirements({&pivot_, &telescope_, &wrist_});
  SetName("intake_command");
}

void IntakeCommand::Initialize() {
  Log("Intake Command Initialize");
}

void IntakeCommand::Execute() {

  if (frc846::util::ShareTables::GetString("arm_position")==""){
    frc846::util::ShareTables::SetString("arm_position", "intake");
  }
  if (frc846::util::ShareTables::GetString("arm_position")=="intake"){
    units::degree_t nextPivotTarget = units::degree_t(setpoints::kIntake(0));
    units::inch_t nextTelescopeTarget = units::inch_t(setpoints::kIntake(1));
    units::degree_t nextWristTarget = units::degree_t(setpoints::kIntake(2));

    bool telescope_in_manual = operator_.readings().a_button;
    bool telescope_out_manual = operator_.readings().b_button;
    bool pivot_down_manual = operator_.readings().x_button;
    bool pivot_up_manual = operator_.readings().y_button;
    bool w_up = operator_.readings().pov == frc846::XboxPOV::kUp;
    bool w_d = operator_.readings().pov == frc846::XboxPOV::kDown;  

    if (w_up) {
        ms_adj -= wrist_.wrist_adj_inc_.value();
    } else if (w_d) {
        ms_adj += wrist_.wrist_adj_inc_.value();
    }

    if (telescope_in_manual) {
        mtele_adj -= telescope_.telescope_adj_inc_.value();
    } else if (telescope_out_manual) {
        mtele_adj += telescope_.telescope_adj_inc_.value();
    }

    if (pivot_up_manual) {
        mpiv_adj += pivot_.pivot_adj_inc_.value();
        ms_adj += wrist_.wrist_adj_inc_.value();
    } else if (pivot_down_manual) {
        mpiv_adj -= pivot_.pivot_adj_inc_.value();
        ms_adj -= wrist_.wrist_adj_inc_.value();
    }


    CoordinatePositions coordinateConverted = InverseKinematics::degree_toCoordinate({nextPivotTarget.to<double>() + mpiv_adj, 
        nextWristTarget.to<double>() + ms_adj, nextTelescopeTarget.to<double>() + mtele_adj});

    CoordinatePositions coordinateConvertedSecond = InverseKinematics::degree_toCoordinatePoint2({nextPivotTarget.to<double>() + mpiv_adj, 
        nextWristTarget.to<double>() + ms_adj, nextTelescopeTarget.to<double>() + mtele_adj});

    // std::cout << "FA" << coordinateConverted.forward_axis << std::endl;
    // std::cout << coordinateConverted.upward_axis << std::endl;

    if (!((InverseKinematics::withinBounds(coordinateConverted) 
        && InverseKinematics::withinBounds(coordinateConvertedSecond)))) {
        mpiv_adj = pmpiv_adj;
        ms_adj = pms_adj;
        mtele_adj = pmtele_adj;

        std::cout << "OUT of bounds, forward " << coordinateConverted.forward_axis << ", upward" << coordinateConverted.upward_axis << std::endl;
    } else {
        pmpiv_adj =  mpiv_adj;
        pms_adj = ms_adj;
        pmtele_adj = mtele_adj;
            
    }

    nextPivotTarget+=units::degree_t(mpiv_adj);
    nextTelescopeTarget+=units::inch_t(mtele_adj);
    nextWristTarget+=units::degree_t(ms_adj);

    pivot_.SetTarget({nextPivotTarget});
    if (pivot_.readings().pivot_position<20.0_deg){
        telescope_.SetTarget({nextTelescopeTarget});
    }
    if (pivot_.readings().pivot_position<20.0_deg){
        wrist_.SetTarget({nextWristTarget});
    }

    //TODO Adjustments

    //TODO: Intermediates?

    bool pivot_done_=units::math::abs(pivot_.readings().pivot_position-nextPivotTarget)<pivot_.pivot_position_tolerance_.value();
    bool telescope_done_=units::math::abs(telescope_.readings().extension-nextTelescopeTarget)<telescope_.telescope_position_tolerance_.value();
    bool wrist_done_=units::math::abs(wrist_.readings().wrist_position-nextWristTarget)<wrist_.wrist_position_tolerance_.value();

    is_done_ = pivot_done_&&telescope_done_&&wrist_done_;
  }
}

void IntakeCommand::End(bool interrupted) {
  Log("Intake Command Finished");
}

bool IntakeCommand::IsFinished() {
  return false;
}