
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/MecanumDrive.h>

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Invert the right side motors. You may need to change or remove this to
    // match your robot.
  }

  void TeleopPeriodic() override {
    int Y = cntrl.GetLeftY();
    if (Y < .1 || Y > -.1){
      Y = 0;
    }
    int X = cntrl.GetLeftX();
    if (X < .1 || X > -.1) {
      X = 0;
    }
    int Z = (cntrl.GetRightTriggerAxis()-cntrl.GetLeftTriggerAxis());
    if (Z < .1 || Z > -.1) {
      Z = 0;
    }
   m_robotDrive.DriveCartesian(Y/4, X/4, Z/4);
  }

 private:
  static constexpr int kFrontLeftChannel = 10;
  static constexpr int kRearLeftChannel = 11;
  static constexpr int kFrontRightChannel = 12;
  static constexpr int kRearRightChannel = 13;

  static constexpr int kControllerChannel = 0;

  rev::CANSparkMax frontLeft{kFrontLeftChannel, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontRight{kFrontRightChannel, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearLeft{kRearLeftChannel, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearRight{kRearRightChannel, rev::CANSparkMax::MotorType::kBrushless};
 frc::MecanumDrive m_robotDrive{frontLeft, rearLeft, frontRight, rearRight};

  frc::XboxController cntrl{kControllerChannel};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif