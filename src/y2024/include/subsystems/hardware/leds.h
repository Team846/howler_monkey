#pragma once

#include <frc/AddressableLED.h>

#include "frc846/subsystem.h"
#include "frc846/util/grapher.h"
#include "ports.h"

enum LEDsState {
  kLEDSNotReady,
  kLEDSZeroing,
  kLEDSTeleop,
  kLEDSAutonomous,
  kLEDSHasPiece,
  kLEDSPreparingShoot,
  kLEDSReadyToShoot,
  kLEDSClimbing,
  kLEDSAmpingLeds,
  kLEDSCOOPLeds,
  kLEDSDisabled,
};

struct LEDsReadings {};

struct LEDsTarget {
  LEDsState state;
};

class LEDsSubsystem : public frc846::Subsystem<LEDsReadings, LEDsTarget> {
 public:
  LEDsSubsystem(bool init);

  void Setup() override {};

  LEDsTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  bool zeroSequence = false;

  // Number of LEDs.
  static constexpr int kLength = 30;

  std::array<frc::AddressableLED::LEDData, kLength> leds_buffer_;

  frc::AddressableLED leds_{ports::leds_::kPWMPort};

  LEDsReadings GetNewReadings() override;

  int loops = 0;
  int first_pixel_hue_ = 0;

  void DirectWrite(LEDsTarget target) override;
};
