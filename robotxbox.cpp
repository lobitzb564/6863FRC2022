
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/PWMSparkMax.h>

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Invert the right side motors. You may need to change or remove this to
    // match your robot.
    m_frontRight.SetInverted(true);
    m_rearRight.SetInverted(true);
  }
 
  void AutonomousInit() override {
    timekeep.Reset();
    timekeep.Start();
  }
  void AutonomousPeriodic() override {
    if (timekeep.Get() > 2_s && timekeep.Get() < 3_s) {
      m_robotDrive.DriveCartesian(0.25, 0, 0);
    } else {
      
    }
  }
  void TeleopPeriodic() override {
    m_robotDrive.DriveCartesian(-cntrl.GetLeftY(), cntrl.GetLeftX(), (cntrl.GetRightTriggerAxis()-cntrl.GetLeftTriggerAxis()));
  }

 private:
  static constexpr int kFrontLeftChannel = 0;
  static constexpr int kRearLeftChannel = 1;
  static constexpr int kFrontRightChannel = 2;
  static constexpr int kRearRightChannel = 3;

  static constexpr int kControllerChannel = 0;

  frc::PWMSparkMax frontLeft{kFrontLeftChannel};
  frc::PWMSparkMax rearLeft{kRearLeftChannel};
  frc::PWMSparkMax frontRight{kFrontRightChannel};
  frc::PWMSparkMax rearRight{kRearRightChannel};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight, m_rearRight};

  frc::XboxController cntrl{kControllerChannel};
 
  frc::Timer timekeep;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
