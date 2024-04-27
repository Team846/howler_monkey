#include "commands/positioning/trap_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

#include "subsystems/setpoints.h"

class TrapCalculator {
  public:
    static std::vector<std::pair<frc846::util::Vector2D<units::inch_t>, units::degree_t>> interpolateTrapPoints(
        frc846::util::Vector2D<units::inch_t> starting_coordinate,
          frc846::util::Vector2D<units::inch_t> ending_coordinate, 
            units::degree_t starting_angle, units::degree_t ending_angle, int steps = 140) {
      std::vector<std::pair<frc846::util::Vector2D<units::inch_t>, units::degree_t>> toReturn{};
      for (int i = 0; i < steps; i++) {
        auto x_coord = starting_coordinate.x + (i * 1.0 / steps) * (ending_coordinate.x - starting_coordinate.x);
        auto y_coord = starting_coordinate.y + (i * 1.0 / steps) * (ending_coordinate.y - starting_coordinate.y);
        auto angle = starting_angle + (i * 1.0 / steps) * (ending_angle - starting_angle);
        toReturn.push_back({{x_coord, y_coord}, angle});

        // std::cout << angle.to<double>() << std::endl;
      }
      return toReturn;
    }

    static RawPositions getRawsAtPoint(int counter, 
      std::vector<std::pair<frc846::util::Vector2D<units::inch_t>, units::degree_t>> trapPoints) {
        auto toReturn =  InverseKinematics::toRaw(CoordinatePositions{radians(trapPoints.at(counter).second.to<double>()), trapPoints.at(counter).first.x.to<double>(), 
          trapPoints.at(counter).first.y.to<double>()});
        toReturn.wrist_angle = radians(trapPoints.at(counter).second.to<double>());
        return toReturn;
    }
};

TrapCommand::TrapCommand(
    RobotContainer& container)
    : frc846::Loggable{"trap_command"}, pivot_(container.pivot_),
        telescope_(container.telescope_), wrist_(container.wrist_), operator_(container.operator_), super_(container.super_structure_) {
  AddRequirements({&pivot_, &telescope_, &wrist_});
  SetName("trap_command");
}

void TrapCommand::Initialize() {
  Log("Trap Command Initialize");
}

void TrapCommand::Execute() { 
    
  if (frc846::util::ShareTables::GetString("arm_position")==""){
    frc846::util::ShareTables::SetString("arm_position", "trap");
  }
  if (frc846::util::ShareTables::GetString("arm_position")=="trap"){   
    auto k = TrapCalculator::interpolateTrapPoints(
            {super_.trap_start_x.value(), super_.trap_start_y.value()},
            {super_.trap_end_x.value(), super_.trap_end_y.value()},
            super_.trap_start_angle.value(),
            super_.trap_end_angle.value());
    RawPositions positions = TrapCalculator::getRawsAtPoint(std::min(139.0, trapCounter/trapDivisor), k);
    // std::cout << trapCounter << "Q" << std::min(39, trapCounter/trapDivisor) << std::endl;
    trapCounter += 1;

    units::degree_t nextPivotTarget = units::degree_t(positions.pivot_angle);
    units::inch_t nextTelescopeTarget = units::inch_t(positions.extension);
    units::degree_t nextWristTarget = units::degree_t(positions.wrist_angle);

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

    // std::cout << "FA" << coordinateConverted.forward_axis << std::endl;
    // std::cout << coordinateConverted.upward_axis << std::endl;

    pmpiv_adj =  mpiv_adj;
    pms_adj = ms_adj;
    pmtele_adj = mtele_adj;

    //   nextPivotTarget+=units::degree_t(mpiv_adj);
    nextTelescopeTarget+=units::inch_t(mtele_adj);
    nextWristTarget+=units::degree_t(ms_adj);

    telescope_.SetTarget({nextTelescopeTarget});
    pivot_.SetTarget({nextPivotTarget});
    wrist_.SetTarget({nextWristTarget});

    //TODO Adjustments

    //TODO: Intermediates?

    bool pivot_done_=units::math::abs(pivot_.readings().pivot_position-nextPivotTarget)<pivot_.pivot_position_tolerance_.value();
    bool telescope_done_=units::math::abs(telescope_.readings().extension-nextTelescopeTarget)<telescope_.telescope_position_tolerance_.value();
    bool wrist_done_=units::math::abs(wrist_.readings().wrist_position-nextWristTarget)<wrist_.wrist_position_tolerance_.value();

    is_done_ = pivot_done_&&telescope_done_&&wrist_done_;
  }
}

void TrapCommand::End(bool interrupted) {
  Log("Trap Command Finished");
}

bool TrapCommand::IsFinished() {
  return false;
}