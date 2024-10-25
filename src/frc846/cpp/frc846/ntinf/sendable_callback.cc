#include <wpi/sendable/SendableBuilder.h>

#include "frc846/ntinf/ntaction.h"

namespace frc846::ntinf {

NTAction::NTAction(std::function<void()> callback) : callback_(callback) {}
void NTAction::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Command");
  builder.AddStringProperty(".name", [] { return "Run"; }, nullptr);
  builder.AddBooleanProperty(
      "running", [] { return false; },
      [this](bool value) {
        if (value) {
          callback_();
        }
      });
}

}  // namespace frc846::ntinf