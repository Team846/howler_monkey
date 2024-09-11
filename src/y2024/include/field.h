#pragma once

#include <cmath>

#include "frc846/math/fieldpoints.h"

// Field has blue alliance far right corner as origin
struct field {
  struct points {
    static frc846::math::FieldPoint Origin() {
      return {{0_in, 0_in}, 0_deg, {0_fps, 0_fps}};
    }

    static frc846::math::VectorND<units::foot_t, 2> kSpeaker(
        bool flip = false) {
      if (!flip) {
        return {217.5_in, -4_in};
      } else {
        return {217.5_in, 655.8_in};
      }
    }

    static frc846::math::FieldPoint kAmpNoFlip() {
      return {{0_in, 0_in}, 90_deg, {0_fps, 0_fps}};
    }

    static frc846::math::FieldPoint kPreAmpNoFlip() {
      return {{-2_ft, 0_in}, 90_deg, {0_fps, 0_fps}};
    }

    // // TESTING
    // static frc846::util::FieldPoint testing_origin;
    // static frc846::util::Position kTestingOrigin(bool should_flip) {
    //   return testing_origin.flip(should_flip);
    // };

    // static frc846::util::FieldPoint testing_point;
    // static frc846::util::Position kTestingPoint(bool should_flip) {
    //   return testing_point.flip(should_flip);
    // };

    // // FIVE PIECE AUTO
    // static frc846::util::FieldPoint five_piece_origin;
    // static frc846::util::Position kFivePieceOrigin(bool should_flip) {
    //   return five_piece_origin.flip(should_flip, true);
    // };

    // static frc846::util::FieldPoint five_piece_intake_one;
    // static frc846::util::Position kFivePieceIntakeOne(bool should_flip) {
    //   return five_piece_intake_one.flip(should_flip, true);
    // };

    // static frc846::util::FieldPoint five_piece_mid_one;
    // static frc846::util::Position kFivePieceMidOne(bool should_flip) {
    //   return five_piece_mid_one.flip(should_flip, true);
    // };

    // static frc846::util::FieldPoint five_piece_shoot_one;
    // static frc846::util::Position kFivePieceShootOne(bool should_flip) {
    //   return five_piece_shoot_one.flip(should_flip, true);
    // };

    // static frc846::util::FieldPoint five_piece_intake_two;
    // static frc846::util::Position kFivePieceIntakeTwo(bool should_flip) {
    //   return five_piece_intake_two.flip(should_flip, true);
    // };

    // static frc846::util::FieldPoint five_piece_mid_two;
    // static frc846::util::Position kFivePieceMidTwo(bool should_flip) {
    //   return five_piece_mid_two.flip(should_flip, true);
    // };

    // static frc846::util::FieldPoint five_piece_shoot_two;
    // static frc846::util::Position kFivePieceShootTwo(bool should_flip) {
    //   return five_piece_shoot_two.flip(should_flip, true);
    // };

    // static frc846::util::FieldPoint five_piece_intake_three;
    // static frc846::util::Position kFivePieceIntakeThree(bool should_flip) {
    //   return five_piece_intake_three.flip(should_flip, true);
    // };

    // static frc846::util::FieldPoint five_piece_mid_three;
    // static frc846::util::Position kFivePieceMidThree(bool should_flip) {
    //   return five_piece_mid_three.flip(should_flip, true);
    // };

    // static frc846::util::FieldPoint five_piece_shoot_three;
    // static frc846::util::Position kFivePieceShootThree(bool should_flip) {
    //   return five_piece_shoot_three.flip(should_flip, true);
    // };
  };
};
