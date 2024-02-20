#ifndef Y2024_SUBSYSTEMS_LEDS_H_
#define Y2024_SUBSYSTEMS_LEDS_H_

#include <frc/AddressableLED.h>

#include "frc846/util/grapher.h"
#include "frc846/subsystem.h"
#include "ports.h"

struct LEDsReadings {};

enum LEDsHumanPlayerState {
  kCoopertition, //purple TODO pick better color
  kAmplify, //white
  kHumanPlayerIdle
};

enum LEDsAutoState {
  kTransit, //flashing red
  kFinished, //green
  kAutoIdle
};

enum LEDsPieceState {
  kHasPiece, //rainbow
  kNoPieceIdle //yellow
};

struct LEDsTarget {
};

class LEDsSubsystem : public frc846::Subsystem<LEDsReadings, LEDsTarget> {
 public:
  LEDsSubsystem(bool init);

  LEDsTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroSubsystem() {
    SetTarget(ZeroTarget());
    hasZeroed = true;
  }

  bool GetHasZeroed() { return hasZeroed; }

 private:
  
  bool hasZeroed = false;
  
  // Number of LEDs.
  static constexpr int kLength = 30;

  std::array<frc::AddressableLED::LEDData, kLength> leds_buffer_;

  frc::AddressableLED leds_{ports::leds_::kPWMPort};

  LEDsReadings GetNewReadings() override;

  int loops = 0;
  int first_pixel_hue_ = 0;

  void DirectWrite(LEDsTarget target) override;
};

#endif