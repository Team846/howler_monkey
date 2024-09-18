#pragma once

#include <cmath>

#include "frc846/math/fieldpoints.h"

// Field has blue alliance far right corner as origin
struct field {
  struct points_cls {
    frc846::math::FieldPoint Origin() {
      return {{0_in, 0_in}, 0_deg, {0_fps, 0_fps}};
    }

    frc846::math::VectorND<units::foot_t, 2> kSpeaker(bool flip = false) {
      if (!flip) {
        return {217.5_in, -4_in};
      } else {
        return {217.5_in, 655.8_in};
      }
    }

    frc846::math::FieldPoint kAmpNoFlip() {
      return {{0_in, 0_in}, 90_deg, {0_fps, 0_fps}};
    }

    frc846::math::FieldPoint kPreAmpNoFlip() {
      return {{-2_ft, 0_in}, 90_deg, {0_fps, 0_fps}};
    }

    // DRIVE AUTO - TEST POINTS

    frc846::math::FieldPointPreference testing_origin{"testing_origin",
                                                      Origin()};
    frc846::math::FieldPoint kTestingOrigin() { return testing_origin.get(); };

    frc846::math::FieldPointPreference testing_point{
        "testing_point", {{0_in, 120_in}, 0_deg, {0_fps, 0_fps}}};
    frc846::math::FieldPoint kTestingPoint() { return testing_point.get(); };

    // FIVE PIECE AUTO
    frc846::base::Loggable five_piece_loggable{"five_piece_auto"};

    frc846::ntinf::Pref<units::foot_t> pre_point_amt{five_piece_loggable,
                                                     "pre_point_amt", 2_ft};
    frc846::math::FieldPoint pre_point(frc846::math::FieldPoint pnt) {
      return {{pnt.point[0], pnt.point[1] - pre_point_amt.value()},
              pnt.bearing,
              {0_fps, 15_fps}};
    }

    frc846::math::FieldPointPreference origin_point{
        "five_piece_origin", {{217.5_in, 49_in}, 0_deg, {0_fps, 0_fps}}};
    frc846::math::FieldPoint kFivePieceOrigin(bool should_flip) {
      return origin_point.get().mirrorOnlyY(should_flip);
    }

    frc846::math::FieldPointPreference intake_one{
        "five_piece_intake_one", {{217.5_in, 112_in}, 0_deg, {0_fps, 0_fps}}};
    std::vector<frc846::math::FieldPoint> intake_one_path(bool should_flip) {
      auto base_point = intake_one.get();
      return {pre_point(base_point).mirrorOnlyY(should_flip),
              base_point.mirrorOnlyY(should_flip)};
    }

    frc846::math::FieldPointPreference intake_two{
        "five_piece_intake_two", {{160.5_in, 112_in}, 0_deg, {0_fps, 0_fps}}};
    std::vector<frc846::math::FieldPoint> intake_two_path(bool should_flip) {
      auto base_point = intake_two.get();
      return {pre_point(base_point).mirrorOnlyY(should_flip),
              base_point.mirrorOnlyY(should_flip)};
    }

    frc846::math::FieldPointPreference intake_three{
        "five_piece_intake_three", {{274.5_in, 112_in}, 0_deg, {0_fps, 0_fps}}};
    std::vector<frc846::math::FieldPoint> intake_three_path(bool should_flip) {
      auto base_point = intake_three.get();
      return {pre_point(base_point).mirrorOnlyY(should_flip),
              base_point.mirrorOnlyY(should_flip)};
    }

    frc846::math::FieldPointPreference finish_pt{
        "five_piece_finish", {{274.5_in, 180_in}, 0_deg, {0_fps, 0_fps}}};
    frc846::math::FieldPoint kFivePieceFinish(bool should_flip) {
      return finish_pt.get().mirrorOnlyY(should_flip);
    }
  };

  static points_cls points;
};
