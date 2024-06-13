#ifndef CONTROL_TRIGGERS_H_
#define CONTROL_TRIGGERS_H_

#include "subsystems/robot_container.h"

class ControlTriggerInitializer {
 public:
  static void InitTeleopTriggers(RobotContainer& container);
};

#endif