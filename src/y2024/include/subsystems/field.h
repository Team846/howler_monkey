#ifndef y2024_FIELD_H_
#define y2024_FIELD_H_

#include <cmath>
#include <cmath>
#include "frc846/util/math.h"

// Field has blue alliance far right corner as origin
struct field {
  struct points {

    static frc846::util::Position Origin() {
      return frc846::util::Position(frc846::util::Vector2D<units::foot_t>(0_in, 0_in), 0_deg);
    };

    static frc846::util::Vector2D<units::foot_t> kSpeaker(bool flip = false) {
      if (!flip) {
        return frc846::util::Vector2D<units::foot_t>(-104.4_in, 14.4_in);
      } else {
        return frc846::util::Vector2D<units::foot_t>(104.4_in, 635.8_in);
      }
    }

    static frc846::util::Vector2D<units::foot_t> kSpeakerTeleop(bool flip = false) {
      if (!flip) {
        return frc846::util::Vector2D<units::foot_t>(104.4_in, 14.4_in);
      } else {
        return frc846::util::Vector2D<units::foot_t>(-104.4_in, 615.8_in);
      }
    }

    //TESTING
    static frc846::util::Position kTestingOrigin(bool should_flip) {
      return frc846::util::FieldPoint("testing_origin", 0_in, 0_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kTestingPoint(bool should_flip) {
      return frc846::util::FieldPoint("testing_point", 0_in, 100_in, 0_deg).flip(should_flip);
    };

    //FIVE PIECE AUTO
    static frc846::util::Position kFPOrigin(bool should_flip) {
      return frc846::util::FieldPoint("FP_start", -163.5_in, 55_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kFPIntakeOne(bool should_flip) {
      return frc846::util::FieldPoint("FP_intake_one", -162.4_in, 111.8_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kFPShootOne(bool should_flip) {
      return frc846::util::FieldPoint("FP_shoot_one", -135.7_in, 86.6_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kFPIntakeTwo(bool should_flip) {
      return frc846::util::FieldPoint("FP_intake_two", -104.7_in, 112.4_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kFPShootTwo(bool should_flip) {
      return frc846::util::FieldPoint("FP_shoot_two", -69.2_in, 85.5_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kFPIntakeThree(bool should_flip) {
      return frc846::util::FieldPoint("FP_intake_three", -45.0_in, 112.9_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kFPShootThree(bool should_flip) {
      return frc846::util::FieldPoint("FP_shoot_three", -58.9_in, 83.3_in, 0_deg).flip(should_flip);
    };
    
    static frc846::util::Position kFPIntakeFour(bool should_flip) {
      return frc846::util::FieldPoint("FP_intake_four", -30.0_in, 324.5_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kFPShootFour(bool should_flip) {
      return frc846::util::FieldPoint("FP_shoot_four", -80.7_in, 84.4_in, 0_deg).flip(should_flip);
    };

    static frc846::util::Position kFPFinalPosition(bool should_flip) {
      return frc846::util::FieldPoint("FP_final_position", -69.2_in, 266.4_in, 0_deg).flip(should_flip);
    };
  };

  struct length {
    static constexpr units::inch_t kFieldLength = 651.25_in;
    static constexpr units::inch_t kFieldWidth = 315.5_in;
  };

};

#endif  // y2024_FIELD_H_