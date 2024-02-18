#include "subsystems/leds.h"

LEDsSubsystem::LEDsSubsystem(bool init)
    : frc846::Subsystem<LEDsReadings, LEDsTarget>("leds", init) {
  if (init) {
    leds_.SetLength(kLength);
    leds_.SetData(leds_buffer_);
    leds_.Start();
  }
}

LEDsTarget LEDsSubsystem::ZeroTarget() const {
  LEDsTarget target;
  target.kHumanPlayer = kHumanPlayerIdle;
  target.kAuto = kAutoIdle;
  target.kPiece = kNoPieceIdle;
  return target;
}

bool LEDsSubsystem::VerifyHardware() { return true; }

LEDsReadings LEDsSubsystem::GetNewReadings() { return {}; }

void LEDsSubsystem::DirectWrite(LEDsTarget target) {
  if (hasZeroed) {
    if (target.kHumanPlayer == kAmplify){
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(255, 255, 255);
      }
    } else if (target.kHumanPlayer == kCoopertition){
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(255, 0, 255);
      }
    } else if (target.kAuto == kTransit){ //TODO flash red
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(200, 50, 50);
      }
    } else if (target.kAuto == kFinished){
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(0, 255, 0);
      }
    } else if (target.kPiece == kHasPiece){
      for (int i = 0; i < kLength; i++) {
        const auto pixelHue = (first_pixel_hue_ + (i * 180 / kLength)) % 180;
        leds_buffer_[i].SetHSV(pixelHue, 255, 128);
      }
      first_pixel_hue_ += 3;
      first_pixel_hue_ %= 180;
    } else {
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(255, 255, 0);
      }
    }
    leds_.SetData(leds_buffer_);
  } else {
    // if (loops % 30 < 15){
    //   for (int i = 0; i < kLength; i++){
    //     leds_buffer_[i].SetRGB(255, 0, 0);
    //   }
    // }
    // else {
    //   for (int i = 0; i < kLength; i++){
    //     leds_buffer_[i].SetRGB(0, 0, 0);
    //   }
    // }
    // loops++;
    // loops%=100;

    for (int i = 0; i < kLength; i++) {
        const auto pixelHue = (first_pixel_hue_ + (i * 180 / kLength)) % 180;
        leds_buffer_[i].SetHSV(pixelHue, 255, 128);
      }
      first_pixel_hue_ += 3;
      first_pixel_hue_ %= 180;
    leds_.SetData(leds_buffer_);
  }
}
