#include "subsystems/hardware/leds.h"

#include "frc846/util/share_tables.h"

LEDsSubsystem::LEDsSubsystem(bool init)
    : frc846::Subsystem<LEDsReadings, LEDsTarget>("leds", init) {
  if (init) {
    leds_.SetLength(kLength);
    leds_.SetData(leds_buffer_);
    leds_.Start();
  }
}

LEDsTarget LEDsSubsystem::ZeroTarget() const { return {kLEDSNotReady}; }

bool LEDsSubsystem::VerifyHardware() { return true; }

LEDsReadings LEDsSubsystem::GetNewReadings() { return {}; }

void LEDsSubsystem::DirectWrite(LEDsTarget target) {
  if (target.state == kLEDSZeroing) {
    if (loops % 18 < 9) {
      for (int i = 0; i < kLength; i++) {
        leds_buffer_[i].SetRGB(255, 17, 0);
      }
    } else {
      for (int i = 0; i < kLength; i++) {
        leds_buffer_[i].SetRGB(0, 0, 0);
      }
    }

    if (loops == 36) {
      frc846::util::ShareTables::SetBoolean("zero sequence", false);
    }

    loops++;
  } else if (target.state == kLEDSCOOPLeds) {
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(255, 255, 255);
    }
  } else if (target.state == kLEDSAmpingLeds) {
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(255, 0, 255);
    }
  } else if (target.state == kLEDSClimbing) {
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(0, 255, 0);
    }
  } else if (target.state == kLEDSPreparingShoot) {
    if (loops % 20 < 10) {
      for (int i = 0; i < kLength; i++) {
        leds_buffer_[i].SetRGB(0, 0, 255);
      }
    } else {
      for (int i = 0; i < kLength; i++) {
        leds_buffer_[i].SetRGB(0, 0, 0);
      }
    }
    loops++;
    loops %= 100;
  } else if (target.state == kLEDSReadyToShoot) {
    if (loops % 10 < 5) {
      for (int i = 0; i < kLength; i++) {
        leds_buffer_[i].SetRGB(0, 255, 0);
      }
    } else {
      for (int i = 0; i < kLength; i++) {
        leds_buffer_[i].SetRGB(0, 0, 0);
      }
    }
    loops++;
    loops %= 100;
  } else if (target.state == kLEDSHasPiece) {
    for (int i = 0; i < kLength; i++) {
      const auto pixelHue = (first_pixel_hue_ + (i * 180 / kLength)) % 180;
      leds_buffer_[i].SetHSV(pixelHue, 255, 128);
    }
    first_pixel_hue_ += 3;
    first_pixel_hue_ %= 180;
  } else if (target.state == kLEDSTeleop) {
    if (loops % 20 < 10) {
      for (int i = 0; i < kLength; i++) {
        leds_buffer_[i].SetRGB(255, 17, 0);
      }
    } else {
      for (int i = 0; i < kLength; i++) {
        leds_buffer_[i].SetRGB(0, 0, 0);
      }
    }
    loops++;
    loops %= 100;
  } else if (target.state == kLEDSAutonomous) {
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(0, 0, 255);
    }
  } else if (target.state == kLEDSNotReady) {
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(255, 0, 0);
    }
  } else {
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(255, 17, 0);
    }
  }

  leds_.SetData(leds_buffer_);
}
