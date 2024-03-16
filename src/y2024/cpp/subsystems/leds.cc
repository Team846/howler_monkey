#include "subsystems/leds.h"
#include "frc846/util/share_tables.h"

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
  return target;
}

bool LEDsSubsystem::VerifyHardware() { return true; }

LEDsReadings LEDsSubsystem::GetNewReadings() { 
  frc846::util::ShareTables::SetBoolean("is_homed", hasZeroed);
  return {}; 
}

void LEDsSubsystem::DirectWrite(LEDsTarget target) {
  if (hasZeroed) {

    if (frc846::util::ShareTables::GetBoolean("zero sequence")){
      //JUST ZEROED
      if (loops % 18 < 9){
        for (int i = 0; i < kLength; i++){
          leds_buffer_[i].SetRGB(255, 255, 0);
        }
      }
      else {
        for (int i = 0; i < kLength; i++){
          leds_buffer_[i].SetRGB(0, 0, 0);
        }
      }

      loops++;
      if (loops > 36) {
        loops = 0;
        frc846::util::ShareTables::SetBoolean("zero sequence", false);
      } 
    }
    else if (frc846::util::ShareTables::GetString("mode").compare("disabled") == 0){ 
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(255, 0, 0);
      }
    } else if (frc846::util::ShareTables::GetBoolean("amp")){
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(255, 255, 255);
      }
    } else if (frc846::util::ShareTables::GetBoolean("coopertition")){
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(255, 0, 255);
      }
    } else if (frc846::util::ShareTables::GetBoolean("is_climb_sequence")){
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(0, 255, 0);
      }
    } else if (frc846::util::ShareTables::GetString("shooting_state").compare("kReady") == 0){
      if (loops % 10 < 5){
        for (int i = 0; i < kLength; i++){
          leds_buffer_[i].SetRGB(0, 255, 0);
        }
      }
      else {
        for (int i = 0; i < kLength; i++){
          leds_buffer_[i].SetRGB(0, 0, 0);
        }
      }
      loops++;
      loops%=100;
    } else if (frc846::util::ShareTables::GetString("shooting_state").compare("kUnready") == 0){
      if (loops % 20 < 10){
        for (int i = 0; i < kLength; i++){
          leds_buffer_[i].SetRGB(0, 0, 255);
        }
      }
      else {
        for (int i = 0; i < kLength; i++){
          leds_buffer_[i].SetRGB(0, 0, 0);
        }
      }
      loops++;
      loops%=100;
    } else if (frc846::util::ShareTables::GetBoolean("scorer_has_piece")){
      for (int i = 0; i < kLength; i++) {
        const auto pixelHue = (first_pixel_hue_ + (i * 180 / kLength)) % 180;
        leds_buffer_[i].SetHSV(pixelHue, 255, 128);
      }
      first_pixel_hue_ += 3;
      first_pixel_hue_ %= 180;
    } else if (frc846::util::ShareTables::GetString("mode").compare("kAutonomous") == 0){
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(0, 0, 255);
      }
    } else {
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(255, 15, 0);
      }
    }
    leds_.SetData(leds_buffer_);
  } else {
    zeroSequence = false;
    if (loops % 30 < 15){
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(255, 0, 0);
      }
    }
    else {
      for (int i = 0; i < kLength; i++){
        leds_buffer_[i].SetRGB(0, 0, 0);
      }
    }
    loops++;
    loops%=120;

    leds_.SetData(leds_buffer_);
  }
}
