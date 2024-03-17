#include "subsystems/field.h"

frc846::util::FieldPoint field::points::testing_origin{"testing_origin", 0_in, 0_in, 0_deg};
frc846::util::FieldPoint field::points::testing_point{"testing_point", 0_in, 100_in, 0_deg};

frc846::util::FieldPoint field::points::op_origin{"op_origin", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::op_end{"op_end", 104.4_in, 100_in, 0_deg};


frc846::util::FieldPoint field::points::four_piece_origin{"four_piece_origin_", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::four_piece_intake_one{"four_piece_intake_one_", 159.4_in, -4.4_in, -30_deg};
frc846::util::FieldPoint field::points::four_piece_shoot_one{"four_piece_shoot_one_", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::four_piece_intake_two{"four_piece_intake_two_", 154.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::four_piece_shoot_two{"four_piece_shoot_two_", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::four_piece_intake_three{"four_piece_intake_three_", 104.4_in, 88.8_in, 30_deg};
frc846::util::FieldPoint field::points::four_piece_shoot_three{"four_piece_shoot_three_", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::four_piece_final_position{"four_piece_final_position", 104.4_in, 124.4_in, 0_deg};

frc846::util::FieldPoint field::points::five_piece_origin{"five_piece_origin_", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::five_piece_intake_one{"five_piece_intake_one_", 159.4_in, -4.4_in, -30_deg};
frc846::util::FieldPoint field::points::five_piece_shoot_one{"five_piece_shoot_one_", 159.4_in, -4.4_in, -25_deg};
frc846::util::FieldPoint field::points::five_piece_intake_center{"five_piece_intake_center_", 177.4_in, 206.2_in, 0_deg};
frc846::util::FieldPoint field::points::five_piece_center_intermediate{"five_piece_center_intermediate_", 159.4_in, -4.4_in, -25_deg};
frc846::util::FieldPoint field::points::five_piece_shoot_center{"five_piece_shoot_center_", 159.4_in, -4.4_in, -35_deg};
frc846::util::FieldPoint field::points::five_piece_intake_two{"five_piece_intake_two_", 177.4_in, 206.2_in, 0_deg};
frc846::util::FieldPoint field::points::five_piece_shoot_two{"five_piece_shoot_two_", 159.4_in, -4.4_in, -35_deg};
frc846::util::FieldPoint field::points::five_piece_three_intermediate{"five_piece_three_intermediate", 159.4_in, -4.4_in, -25_deg};
frc846::util::FieldPoint field::points::five_piece_intake_three{"five_piece_intake_three_", 159.4_in, -4.4_in, -25_deg};
frc846::util::FieldPoint field::points::five_piece_shoot_three{"five_piece_shoot_three_", 159.4_in, -4.4_in, -25_deg};


frc846::Loggable& point_names_ = *(new frc846::Loggable("Preferences/field_points"));
frc846::Pref<units::foot_t> field::points::one_piece_extra_distance_{point_names_, "one_piece_extra_distance", 5_ft};