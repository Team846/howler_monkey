#ifndef y2024_FUNKY_ROBOT_H_
#define y2024_FUNKY_ROBOT_H_

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <hal/Types.h>
#include <units/time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cameraserver/CameraServer.h>

#include "frc846/util/pref.h"
#include "autos/drive_auto.h"
#include "autos/five_piece_auto.h"
#include "subsystems/robot_container.h"
#include "frc/DigitalInput.h"
#include "frc846/fstore.h"

enum Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

class FunkyRobot : public frc::RobotBase, public frc846::Loggable {
 public:
  static constexpr auto kPeriod = 20_ms;  // 50hz

  FunkyRobot();

  ~FunkyRobot() override;

  void StartCompetition() override;
  void EndCompetition() override;

  void VisionThread(){
// Get the USB camera from CameraServer
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(240, 240);

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::PutVideo("Rectangle", 240, 240);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;

    while (true) {
      // Tell the CvSink to grab a frame from the camera and
      // put it
      // in the source mat.  If there is an error notify the
      // output.
      if (cvSink.GrabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.NotifyError(cvSink.GetError());
        // skip the rest of the current iteration
        continue;
      }
      // Give the output stream a new image to display
      outputStream.PutFrame(mat);
    }
  }

  void InitTeleopDefaults();
  void InitTeleopTriggers();

  void InitTestDefaults();

  void VerifyHardware();

 private:
  hal::Handle<HAL_NotifierHandle> notifier_;
  units::microsecond_t next_loop_time_;

  Mode last_mode_;

 private:
  RobotContainer container_;

  frc846::Grapher<int> time_remaining_graph_{*this, "time"};

  frc846::Grapher<int> warnings_graph_{*this, "warnings"};
  frc846::Grapher<int> errors_graph_{*this, "errors"};

  frc846::Grapher<double> can_usage_graph_{*this, "CAN_usage"};
  frc846::Grapher<units::millisecond_t> loop_time_graph_{*this, "loop_time"};

  frc2::Command* auto_command_ = nullptr;
  frc::SendableChooser<frc2::Command*> auto_chooser_;

  // TODO: Find fix for this
  // Seperate blue vs red because command is generated prior to the alliance
  // color being changed

  // Autos
  frc2::CommandPtr drive_auto_ =
      DriveAuto{container_, true}.ToPtr();
  
  frc2::CommandPtr five_piece_auto_ = 
      FivePieceAuto{container_, false}.ToPtr();

  frc::DigitalInput homing_switch_{0};

  frc846::FunkyStore robotStore {};
};

#endif  // y2024_FUNKY_ROBOT_H_