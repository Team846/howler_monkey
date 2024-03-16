#include "subsystems/field.h"

frc846::util::FieldPoint field::points::testing_origin{"testing_origin", 0_in, 0_in, 0_deg};
frc846::util::FieldPoint field::points::testing_point{"testing_point", 0_in, 100_in, 0_deg};

frc846::util::FieldPoint field::points::op_origin{"op_origin", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::op_end{"op_end", 104.4_in, 100_in, 0_deg};

frc846::util::FieldPoint field::points::fp_start{"fp_start_", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::fp_intake_one{"fp_intake_one_", 159.4_in, -4.4_in, -30_deg};
frc846::util::FieldPoint field::points::fp_shoot_one{"fp_shoot_one_", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::fp_intake_two{"fp_intake_two_", 154.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::fp_shoot_two{"fp_shoot_two_", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::fp_intake_three{"fp_intake_three_", 104.4_in, 88.8_in, 30_deg};
frc846::util::FieldPoint field::points::fp_shoot_three{"fp_shoot_three_", 104.4_in, 44.4_in, 0_deg};
frc846::util::FieldPoint field::points::fp_intake_four{"fp_intake_four_", 104.4_in, 88.8_in, 30_deg};
frc846::util::FieldPoint field::points::fp_shoot_four{"fp_shoot_four_", 104.4_in, 44.4_in, 0_deg};

frc846::util::FieldPoint field::points::fp_final_position{"fp_final_position", 104.4_in, 124.4_in, 0_deg};

frc846::Loggable& point_names_ = *(new frc846::Loggable("Preferences/field_points"));
frc846::Pref<units::foot_t> field::points::one_piece_extra_distance_{point_names_, "one_piece_extra_distance", 5_ft};