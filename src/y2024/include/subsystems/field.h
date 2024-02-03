#ifndef y2024_FIELD_H_
#define y2024_FIELD_H_

#include <cmath>
#include <cmath>
#include "frc846/math.h"

// Field has blue alliance far right corner as origin
struct field {
  struct points {

    static frc846::Position Origin() {
      return frc846::Position(frc846::Vector2D<units::foot_t>(0_in, 0_in), 0_deg);
    };

    static frc846::Vector2D<units::foot_t> kSpeaker() {
      return frc846::Vector2D<units::foot_t>(0_in, 240_in);
    }

    //TESTING
    static frc846::Position kTestingOrigin(bool should_flip) {
      return frc846::FieldPoint("testing_origin", 0_in, 0_in, 0_deg).flip(should_flip);
    };

    static frc846::Position kTestingPoint(bool should_flip) {
      return frc846::FieldPoint("testing_point", 0_in, 100_in, 0_deg).flip(should_flip);
    };

    //FIVE PIECE AUTo
    static frc846::Position kFPOrigin(bool should_flip) {
      if (should_flip) return frc846::Position(frc846::Vector2D<units::foot_t>(50_in, 160_in), 180_deg);
      return frc846::Position(frc846::Vector2D<units::foot_t>(270_in, 160_in), 0_deg);
    };

    static frc846::Position kFPPreloadShoot(bool should_flip) {
      return frc846::FieldPoint("fp_preload_shoot", kFPOrigin(should_flip).point.x, 
            kFPOrigin(should_flip).point.y, 72.7_deg).flip(should_flip);
    };

    static frc846::Position kFPWing3Intake(bool should_flip) {
      return frc846::FieldPoint("fp_wing_three_intake", 120_in, 160_in, 180_deg).flip(should_flip);
    };

    static frc846::Position kFPWing3Shoot(bool should_flip) {
      return frc846::FieldPoint("fp_wing_3_shoot", kFPWing3Intake(should_flip).point.x - 15_in, 
            kFPWing3Intake(should_flip).point.y + 15_in, 53.3_deg).flip(should_flip);
    };

    static frc846::Position kFPWing2Intake(bool should_flip) {
      return frc846::FieldPoint("fp_wing_2_intake", 120_in, 219_in, 88_deg).flip(should_flip);
    };

    static frc846::Position kFPWing2Shoot(bool should_flip) {
      return frc846::FieldPoint("fp_wing_2_shoot", kFPWing2Intake(should_flip).point.x, 
            kFPWing2Intake(should_flip).point.y, 0_deg).flip(should_flip);
    };

    static frc846::Position kFPWing1Intake(bool should_flip) {
      return frc846::FieldPoint("fp_wing_1_intake", 120_in, 245_in, 90_deg).flip(should_flip);
    };

    static frc846::Position kFPWing1Shoot(bool should_flip) {
      return frc846::FieldPoint("fp_wing_1_shoot", kFPWing1Intake(should_flip).point.x, 
            kFPWing1Intake(should_flip).point.y, 310_deg).flip(should_flip);
    };
    
    static frc846::Position kFPCenter1Intake(bool should_flip) {
      return frc846::FieldPoint("fp_center_1_intake", 310_in, 270_in, 180_deg).flip(should_flip);
    };

    static frc846::Position kFPCenter1Shoot(bool should_flip) {
      return frc846::FieldPoint("fp_center_1_shoot", 130_in, 
            260_in, 330_deg).flip(should_flip);
    };


    //FOUR PIECE TOP SWEEP AUTO
    static frc846::Position kTSOrigin(bool should_flip) {
      if (should_flip) return frc846::Position(frc846::Vector2D<units::foot_t>(50_in, 245_in), 180_deg);
      return frc846::Position(frc846::Vector2D<units::foot_t>(270_in, 245_in), 0_deg);
    };

    static frc846::Position kTSPreloadShoot(bool should_flip) {
      return frc846::FieldPoint("ts_preload_shoot", kTSOrigin(should_flip).point.x, 
            kTSOrigin(should_flip).point.y, 310_deg).flip(should_flip);
    };

    static frc846::Position kTSWing1Intake(bool should_flip) {
      return frc846::FieldPoint("ts_wing_three_intake", 120_in, 245_in, 180_deg).flip(should_flip);
    };

    static frc846::Position kTSWing1Shoot(bool should_flip) {
      return frc846::FieldPoint("ts_wing_3_shoot", kFPWing3Intake(should_flip).point.x, 
            kFPWing3Intake(should_flip).point.y, 300_deg).flip(should_flip);
    };

    static frc846::Position kTSWing2Intake(bool should_flip) {
      return frc846::FieldPoint("ts_wing_2_intake", 120_in, 219_in, 270_deg).flip(should_flip);
    };

    static frc846::Position kTSWing2Shoot(bool should_flip) {
      return frc846::FieldPoint("ts_wing_2_shoot", kFPWing2Intake(should_flip).point.x, 
            kFPWing2Intake(should_flip).point.y, 0_deg).flip(should_flip);
    };

    static frc846::Position kTSWing3Intake(bool should_flip) {
      return frc846::FieldPoint("ts_wing_1_intake", 115_in, 165_in, 260_deg).flip(should_flip);
    };

    static frc846::Position kTSWing3Shoot(bool should_flip) {
      return frc846::FieldPoint("ts_wing_1_shoot", kFPWing1Intake(should_flip).point.x, 
            kFPWing1Intake(should_flip).point.y, 53.3_deg).flip(should_flip);
    };

  };

  struct length {
    static constexpr units::inch_t kFieldLength = 651.25_in;
    static constexpr units::inch_t kFieldWidth = 315.5_in;
  };

};

#endif  // y2024_FIELD_H_