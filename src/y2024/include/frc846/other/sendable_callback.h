#pragma once

#include <wpi/sendable/Sendable.h>

#include <functional>

namespace frc846::other {

// Like an instant command but works when robot is disabled.
//
// Use with SmartDashboard::PutData to have buttons call functions on
// Shuffleboard.
class SendableCallback : public wpi::Sendable {
 public:
  SendableCallback(std::function<void()> callback);

  void InitSendable(wpi::SendableBuilder& builder);

 private:
  std::function<void()> callback_;
};

}  // namespace frc846::other
