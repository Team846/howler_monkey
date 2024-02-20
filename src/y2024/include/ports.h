#ifndef y2024_PORTS_H_
#define y2024_PORTS_H_

struct ports {
  struct driver_ {
    static constexpr int kXbox_DSPort = 0;
  };

  struct operator_ {
    static constexpr int kXbox_DSPort = 1;
  };

  struct drivetrain_ { //TODO: change all ports (preferably, when setting them, make them ascending in this order)
    static constexpr int kFLDrive_CANID = 12;
    static constexpr int kFRDrive_CANID = 9;
    static constexpr int kBLDrive_CANID = 6;
    static constexpr int kBRDrive_CANID = 3;

    static constexpr int kFLSteer_CANID = 11;
    static constexpr int kFRSteer_CANID = 8;
    static constexpr int kBLSteer_CANID = 5;
    static constexpr int kBRSteer_CANID = 2;

    static constexpr int kFLCANCoder_CANID = 13;
    static constexpr int kFRCANCoder_CANID = 10;
    static constexpr int kBLCANCoder_CANID = 7;
    static constexpr int kBRCANCoder_CANID = 4;
  };

  struct scorer_ {
    static constexpr int kController_CANID = 20;
  };

  struct positioning_ {
    static constexpr int kPivotOne_CANID = 25;
    static constexpr int kPivotTwo_CANID = 26;
    static constexpr int kPivotThree_CANID = 27;
    static constexpr int kPivotFour_CANID = 28;

    static constexpr int kWrist_CANID = 17;

    static constexpr int kTele1_CANID = 31;
    static constexpr int kTele2_CANID = 32;
  };

  struct bracer_ {
    static constexpr int kPWM_Servo_Left = 3;
    static constexpr int kPWM_Servo_Right = 4;
  };

  struct leds_ {
    static constexpr int kPWMPort = 6;
  };
};

#endif  // y2024_PORTS_H_