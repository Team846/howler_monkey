#ifndef y2024_FIELD_H_
#define y2024_FIELD_H_

#include <cmath>
#include <cmath>
#include "frc846/util/math.h"

// Field has blue alliance far right corner as origin
struct field {
  struct points {

    static frc846::util::Position Origin() {
      return {{0_in, 0_in}, 0_deg};
    };

    static frc846::util::Vector2D<units::foot_t> kSpeaker(bool flip = false) {
      if (!flip) {
        return {-100.4_in, 0_in};
      } else {
        return {-100.4_in, 659.8_in};
      }
    }

    static frc846::util::Vector2D<units::foot_t> kSpeakerTeleop(bool flip = false) {
      if (!flip) {
        return {-100.4_in, -10.4_in};
      } else {
        return {-100.4_in, 659.8_in};
      }
    }

    //TESTING
    static frc846::util::FieldPoint testing_origin;
    static frc846::util::Position kTestingOrigin(bool should_flip) {
      return testing_origin.flip(should_flip);
    };

    static frc846::util::FieldPoint testing_point;
    static frc846::util::Position  kTestingPoint(bool should_flip) {
      return testing_point.flip(should_flip);
    };

    //FOUR PIECE AUTO
    static frc846::util::FieldPoint four_piece_origin;
    static frc846::util::Position kFourPieceOrigin(bool should_flip) {
      return four_piece_origin.flip(should_flip);
    };

    static frc846::util::FieldPoint four_piece_intake_one;
    static frc846::util::Position kFourPieceIntakeOne(bool should_flip) {
      return four_piece_intake_one.flip(should_flip);
    };

    static frc846::util::FieldPoint four_piece_shoot_one;
    static frc846::util::Position kFourPieceShootOne(bool should_flip) {
      return four_piece_shoot_one.flip(should_flip);
    };

    static frc846::util::FieldPoint four_piece_intake_two;
    static frc846::util::Position kFourPieceIntakeTwo(bool should_flip) {
      return four_piece_intake_two.flip(should_flip);
    };

    static frc846::util::FieldPoint four_piece_shoot_two;
    static frc846::util::Position kFourPieceShootTwo(bool should_flip) {
      return four_piece_shoot_two.flip(should_flip);
    };

    static frc846::util::FieldPoint four_piece_intake_three;
    static frc846::util::Position kFourPieceIntakeThree(bool should_flip) {
      return four_piece_intake_three.flip(should_flip);
    };

    static frc846::util::FieldPoint four_piece_shoot_three;
    static frc846::util::Position kFourPieceShootThree(bool should_flip) {
      return four_piece_shoot_three.flip(should_flip);
    };

    static frc846::util::FieldPoint four_piece_final_position;
    static frc846::util::Position kFourPieceFinalPosition(bool should_flip) {
      return four_piece_final_position.flip(should_flip);
    };


    //FIVE PIECE AUTO
    
    static frc846::util::FieldPoint five_piece_origin;
    static frc846::util::Position kFivePieceOrigin(bool should_flip) {
      return five_piece_origin.flip(should_flip);
    };

    static frc846::util::FieldPoint five_piece_intake_one;
    static frc846::util::Position kFivePieceIntakeOne(bool should_flip) {
      return five_piece_intake_one.flip(should_flip);
    };

    static frc846::util::FieldPoint five_piece_shoot_one;
    static frc846::util::Position kFivePieceShootOne(bool should_flip) {
      return five_piece_shoot_one.flip(should_flip);
    };

    static frc846::util::FieldPoint five_piece_intake_center;
    static frc846::util::Position kFivePieceIntakeCenter(bool should_flip) {
      return five_piece_intake_center.flip(should_flip);
    };

    static frc846::util::FieldPoint five_piece_center_intermediate;
    static frc846::util::Position kFivePieceCenterIntermediate(bool should_flip) {
      return five_piece_center_intermediate.flip(should_flip);
    };

    static frc846::util::FieldPoint five_piece_shoot_center;
    static frc846::util::Position kFivePieceShootCenter(bool should_flip) {
      return five_piece_shoot_center.flip(should_flip);
    };

    static frc846::util::FieldPoint five_piece_intake_two;
    static frc846::util::Position kFivePieceIntakeTwo(bool should_flip) {
      return five_piece_intake_two.flip(should_flip);
    };

    static frc846::util::FieldPoint five_piece_shoot_two;
    static frc846::util::Position kFivePieceShootTwo(bool should_flip) {
      return five_piece_shoot_two.flip(should_flip);
  };

    static frc846::util::FieldPoint five_piece_three_intermediate;
    static frc846::util::Position kFivePieceThreeIntermediate(bool should_flip) {
      return five_piece_three_intermediate.flip(should_flip);
    };

    static frc846::util::FieldPoint five_piece_intake_three;
    static frc846::util::Position kFivePieceIntakeThree(bool should_flip) {
      return five_piece_intake_three.flip(should_flip);
    };

    static frc846::util::FieldPoint five_piece_shoot_three;
    static frc846::util::Position kFivePieceShootThree(bool should_flip) {
      return five_piece_shoot_three.flip(should_flip);
    };

    // THREE PIECE SOURCE AUTO
    static frc846::util::Position kSSOrigin(bool should_flip) {
      return frc846::util::FieldPoint("SS_start", -163.5_in, 55_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kSSIntakeOne(bool should_flip) {
      return frc846::util::FieldPoint("SS_intake_one", -30.0_in, 324.5_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kSSShootOne(bool should_flip) {
      return frc846::util::FieldPoint("SS_shoot_one", -80.7_in, 84.4_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kSSIntakeTwo(bool should_flip) {
      return frc846::util::FieldPoint("SS_intake_two", -63.0_in, 324.5_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kSSShootTwo(bool should_flip) {
      return frc846::util::FieldPoint("SS_shoot_twp", -80.7_in, 84.4_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kSSFinalPosition(bool should_flip) {
      return frc846::util::FieldPoint("SS_final_position", -139.2_in, 266.4_in, 0_deg).flip(should_flip);
    };

    static frc846::Pref<units::foot_t> one_piece_extra_distance_;
    
    //ONE PIECE AUTO
    static frc846::util::FieldPoint op_origin;
    static frc846::util::Position kOPOrigin(bool should_flip){
      return op_origin.flip(should_flip);
    }

    static frc846::util::FieldPoint op_end;
    static frc846::util::Position kOPEnd(bool should_flip){
      return op_end.flip(should_flip);
    }
  };

  struct length {
    static constexpr units::inch_t kFieldLength = 651.25_in;
    static constexpr units::inch_t kFieldWidth = 315.5_in;
  };

};

#endif  // y2024_FIELD_H_