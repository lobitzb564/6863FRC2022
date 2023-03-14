#define PI           3.14159265358979323846
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>
#include <frc/AnalogEncoder.h>
#include <cameraserver/CameraServer.h>
#include<frc/Timer.h>
#include <units/velocity.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/interfaces/Gyro.h>
#include <frc/PneumaticsModuleType.h>
#include <rev/SparkMaxPIDController.h>
#include <frc/PneumaticHub.h>
#include <frc/DoubleSolenoid.h>
#include <rev/ControlType.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with split arcade steering and an Xbox controller.
 */
class Robot : public frc::TimedRobot {
  frc::Translation2d m_frontLeftLocation{.267_m, .365_m};
  frc::Translation2d m_frontRightLocation{.267_m, -0.365_m};
  frc::Translation2d m_backLeftLocation{-0.267_m, 0.365_m};
  frc::Translation2d m_backRightLocation{-0.267_m, -0.365_m};


  rev::CANSparkMax wheelfl{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::SparkMaxPIDController pidfl = wheelfl.GetPIDController();
  rev::CANSparkMax wheelfr{5, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::SparkMaxPIDController pidfr = wheelfr.GetPIDController();
  rev::CANSparkMax wheelbl{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::SparkMaxPIDController pidbl = wheelbl.GetPIDController();
  rev::CANSparkMax wheelbr{7, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::SparkMaxPIDController pidbr = wheelbr.GetPIDController();
  
  // for encoders, consider changing methods to GetAlternateEncoder with AlternateEncoder::Type::kHallEffect or something if you face an error
  //rev::CANSparkMax rotfl{10, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax rotfl{2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encfl {0};
  rev::CANSparkMax rotfr{6, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encfr {1};
  rev::CANSparkMax rotbl{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encbl {2};
  rev::CANSparkMax rotbr{8, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encbr {3};
  bool pneumaticson = true;
  int abuttoncount = 0;
  double speedfactor = 2000;
  double getto = 3000;
  double changing = false;
  double cfac = 0;
  frc::Timer time;
  //frc::PneumaticHub pcontrol {15};
  frc::DoubleSolenoid grippercontrol {15, frc::PneumaticsModuleType::REVPH, 0, 1};
  rev::CANSparkMax armmtr {10, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::BuiltInAccelerometer acc;
  
//rev::CANSparkMax testmtr{10, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
//rev::SparkMaxPIDController pidController = testmtr.GetPIDController();

 
// Creating my kinematics object using the module locations.
frc::SwerveDriveKinematics<4> kinematics{
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
  m_backRightLocation};
  frc::XboxController controller{0};
  frc::XboxController controller2{1};
 public:
  void RobotInit() override {

  wheelfl.SetInverted(true);
  wheelbr.SetInverted(true);
  wheelbl.SetInverted(true);
  pidfl.SetP(0.0001);
    pidfl.SetI(.000001);
    pidfl.SetD(0.00000001);

    pidfr.SetP(0.0001);
    pidfr.SetI(.000001);
    pidfr.SetD(0.00000001);

    pidbl.SetP(0.0001);
    pidbl.SetI(.000001);
    pidbl.SetD(0.00000001);

    pidbr.SetP(0.0001);
    pidbr.SetI(.000001);
    pidbr.SetD(0.00000001);
// set proper motors inverted here
  }
  void AutonomousInit() override {
    time.Start();
    frc::CameraServer::StartAutomaticCapture();
  }

  void AutonomousPeriodic() override {

    // absolute encoders are analog and measure position in rotations
    /*
    if (time.Get() < 3_s) {
        Drive(0, 1, 0);
     } else {
      
    if (acc.GetY() <= -.1) {
        Drive(0, acc.GetY(), 0);
      } else if (acc.GetY() >= .1) {
        Drive(0, acc.GetY(), 0);
      }
     }
     */
      
     if (time.Get() <= 3.1_s) {
        armmtr.Set(-.5);
     } else if (time.Get() < 4_s){
        armmtr.Set(0);
     } else if (time.Get() < 5.5_s) {
      AutoDrive(0, -.6, 0);
      // this is where the arm should open, if it doesn't change kForward to kReverse
        
     } else if (time.Get() < 6_s) {
      AutoDrive(0, 0, 0);
     } else if (time.Get() < 8_s) {
        grippercontrol.Set(frc::DoubleSolenoid::kReverse);
     } else if (time.Get() < 12.5_s) {
        AutoDrive(0, .8, 0);
     } else if (time.Get() < 14_s) {
      AutoDrive(0, 0, 0);
      armmtr.Set(.3);
     }

  }

  void TeleopInit() override {
    time.Stop();
    time.Reset();
    pneumaticson = true;
    speedfactor = 2000;
    frc::CameraServer::StartAutomaticCapture();
  }

  void Drive(double x, double y, double rotate) {
    frc::Rotation2d rot2d;
    units::radians_per_second_t rad {rotate};
    units::meters_per_second_t speedy {y};
    units::meters_per_second_t speedx {x};
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speedy, speedx, rad, rot2d);

    auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

    // move swerve motors to angles here

    double getfl = fl.angle.Radians().value()/2.0/PI;
    double getfr = fr.angle.Radians().value()/2.0/PI;
    double getbl = bl.angle.Radians().value()/2.0/PI;
    double getbr = br.angle.Radians().value()/2.0/PI;




    double frole = fl.speed.value();
    double frori = fr.speed.value();
    double bale = bl.speed.value();
    double bari = br.speed.value();
    
    double flpos = encfl.GetAbsolutePosition()-getfl;
    
    if (flpos > 0.5) {
      flpos -= 1;
    } else if (flpos < -0.5) {
      flpos+=1;
    }
    
    double frpos = fmod(encfr.GetAbsolutePosition()+.14, 1)-getfr;
    if (frpos > 0.5) {
      frpos -= 1;
    } else if (frpos < -0.5) {
      frpos+=1;
    }
    double blpos = fmod(encbl.GetAbsolutePosition()+.4, 1)-getbl;
    if (blpos > 0.5) {
      blpos -= 1;
    } else if (blpos < -0.5) {
      blpos+=1;
    }
    
    double brpos = encbr.GetAbsolutePosition()-getbr;
    if (brpos > 0.5) {
      brpos -= 1;
    } else if (brpos < -0.5) {
      brpos+=1;
    }
    
    rotfl.Set(flpos*1.5);
    rotfr.Set(frpos*1.5);
    rotbl.Set(blpos*1.5);
    rotbr.Set(brpos*1.5);
    
  
    pidfl.SetReference(frole*speedfactor, rev::ControlType::kVelocity);
  
    pidfr.SetReference(frori*speedfactor, rev::ControlType::kVelocity);
    
    pidbl.SetReference(bale*speedfactor, rev::ControlType::kVelocity);
   
    pidbr.SetReference(bari*speedfactor, rev::ControlType::kVelocity);
  }
  void TeleopPeriodic() override {
    

    frc::SmartDashboard::PutNumber("x", acc.GetX());
    frc::SmartDashboard::PutNumber("y", acc.GetY());
    frc::SmartDashboard::PutNumber("z", acc.GetZ());
    if (controller2.GetAButtonPressed()) {
      pneumaticson = !pneumaticson;
      if (pneumaticson) {
        grippercontrol.Set(frc::DoubleSolenoid::kReverse);
      } else {
        grippercontrol.Set(frc::DoubleSolenoid::kForward);
      }
      abuttoncount += 1;
    }


  /*if (changing) {
    if (cfac > 0) {
      if (speedfactor >= getto) {
        changing = false;
      } else {
        speedfactor += cfac;
      }
    } else {
      if (speedfactor <= getto) {
        changing = false;
      } else {
        speedfactor += cfac;
      }
    }
  } else {*/
    if (controller.GetRightBumper()) {
      if (speedfactor < 4000) {
        speedfactor += 20;
      }
    } else if (controller.GetLeftBumper()) {
      if (speedfactor > 1000) {
        speedfactor -= 20;
      }
    }
  //}
    
    double x = controller.GetLeftX();
    if (x < 0.1 && x > -.1) {
      x = 0;
    }
    double y = controller.GetLeftY();
    if (y < 0.1 && y > -0.1) {
      y = 0;
    }
    double turn = controller.GetRightX();
    if (turn < 0.1 && turn > -0.1) {
      turn = 0;
    }
    double armright = controller2.GetRightTriggerAxis();
    double armleft = controller2.GetLeftTriggerAxis();
   // frc::SmartDashboard::PutNumber("left", pcontrol.GetSolenoids());
    armmtr.Set((armleft-armright)/2);

    units::velocity::meters_per_second_t contrx {x};
    units::velocity::meters_per_second_t contry {y};
    units::radians_per_second_t rot {turn};

    // absolute encoders are analog and measure position in rotations
    if (controller.GetBButton()) {
      AutoDrive(1, 0, 0);
    } else if (controller.GetXButton()) {
      AutoDrive(-1, 0, 0);
    } else if (controller.GetAButton()) {
      AutoDrive(0,1, 0);
    } else if (controller.GetYButton()) {
      AutoDrive(0,-1,0);
    } else {
      Drive(x, y, turn);
    }

  }

  void AutoDrive(double x, double y, double rotate) {
    frc::Rotation2d rot2d;
    units::radians_per_second_t rad {rotate};
    units::meters_per_second_t speedy {y};
    units::meters_per_second_t speedx {x};
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speedy, speedx, rad, rot2d);

    auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

    // move swerve motors to angles here

    double getfl = fl.angle.Radians().value()/2.0/PI;
    double getfr = fr.angle.Radians().value()/2.0/PI;
    double getbl = bl.angle.Radians().value()/2.0/PI;
    double getbr = br.angle.Radians().value()/2.0/PI;




    double frole = fl.speed.value();
    double frori = fr.speed.value();
    double bale = bl.speed.value();
    double bari = br.speed.value();
    
    double flpos = encfl.GetAbsolutePosition()-getfl;
    

    if (flpos > 0.5) {
      flpos -= 1;
    } else if (flpos < -0.5) {
      flpos+=1;
    }
    
    double frpos = fmod(encfr.GetAbsolutePosition()+.14, 1)-getfr;
    if (frpos > 0.5) {
      frpos -= 1;
    } else if (frpos < -0.5) {
      frpos+=1;
    }
    double blpos = fmod(encbl.GetAbsolutePosition()+.4, 1)-getbl;
    if (blpos > 0.5) {
      blpos -= 1;
    } else if (blpos < -0.5) {
      blpos+=1;
    }
    
    double brpos = encbr.GetAbsolutePosition()-getbr;
    if (brpos > 0.5) {
      brpos -= 1;
    } else if (brpos < -0.5) {
      brpos+=1;
    }
    
    rotfl.Set(flpos*1.5);
    rotfr.Set(frpos*1.5);
    rotbl.Set(blpos*1.5);
    rotbr.Set(brpos*1.5);

    if (brpos < .05 && blpos < .05 && flpos < .05 && frpos < .05 && frpos > -.05 && brpos > -.05 && blpos > -.05 && flpos > -.05) {
      pidfl.SetReference(frole*speedfactor, rev::ControlType::kVelocity);
  
      pidfr.SetReference(frori*speedfactor, rev::ControlType::kVelocity);
    
      pidbl.SetReference(bale*speedfactor, rev::ControlType::kVelocity);
   
      pidbr.SetReference(bari*speedfactor, rev::ControlType::kVelocity);
    } else {
      pidfl.SetReference(0, rev::ControlType::kVelocity);
  
      pidfr.SetReference(0, rev::ControlType::kVelocity);
    
      pidbl.SetReference(0, rev::ControlType::kVelocity);
   
      pidbr.SetReference(0, rev::ControlType::kVelocity);
    }
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif