
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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
#include <units/velocity.h>
#include <frc/AnalogGyro.h>
#include <rev/SparkMaxPIDController.h>
#include <frc/PneumaticsControlModule.h>
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
  bool pneumaticson = false;
  frc::PneumaticsControlModule pcontrol {15};
  frc::DoubleSolenoid grippercontrol = pcontrol.MakeDoubleSolenoid(0, 1);
  rev::CANSparkMax armmtr {10, rev::CANSparkMaxLowLevel::MotorType::kBrushless};


//rev::CANSparkMax testmtr{10, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
//rev::SparkMaxPIDController pidController = testmtr.GetPIDController();

 
// Creating my kinematics object using the module locations.
frc::SwerveDriveKinematics<4> kinematics{
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
  m_backRightLocation};
  frc::XboxController controller{0};
 public:
  void RobotInit() override {

  wheelfl.SetInverted(true);
  wheelbr.SetInverted(true);
  wheelbl.SetInverted(true);
    //pidController.SetP(.0001);
    //pidController.SetI(.000001);
    //pidController.SetD(0.00000001);
    

    //pidController.SetIZone(0);
    //pidController.SetFF(0);
    //pidController.SetOutputRange(-1, 1);

// set proper motors inverted here
  }
  void AutonomousInit() override {
    //pidController.SetReference(speed, rev::ControlType::kVelocity);
    rotfl.Set(0);
    rotfr.Set(0);
    rotbl.Set(0);
    rotbr.Set(0);
    encfl.Reset();
    encfr.Reset();
    encbl.Reset();
    encbr.Reset();
    //testmtr.Set(.05);
  }

  void AutonomousPeriodic() override {
      frc::SmartDashboard::PutNumber("frontleft", encfl.GetAbsolutePosition());
frc::SmartDashboard::PutNumber("frontright", encfr.GetAbsolutePosition());
frc::SmartDashboard::PutNumber("backleft", encbl.GetAbsolutePosition());
frc::SmartDashboard::PutNumber("backright", encbr.GetAbsolutePosition());
  }

  void TeleopInit() override {
    pneumaticson = false;
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


  }
  void TeleopPeriodic() override {
    if (controller.GetAButtonPressed()) {
      grippercontrol.Toggle();
    }
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

    double armright = controller.GetRightTriggerAxis();
    double armleft = controller.GetLeftTriggerAxis();
    frc::SmartDashboard::PutNumber("left", armleft);
    frc::SmartDashboard::PutNumber("right", armright);
    armmtr.Set((armright-armleft)/2);

    units::velocity::meters_per_second_t contrx {x};
    units::velocity::meters_per_second_t contry {y};
    units::radians_per_second_t rot {turn};

    frc::Rotation2d rot2d;
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(contry, contrx, rot*2, rot2d);

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

    // absolute encoders are analog and measure position in rotations

    
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
    
    rotfl.Set(flpos);
    rotfr.Set(frpos);
    rotbl.Set(blpos);
    rotbr.Set(brpos);
    
  
    pidfl.SetReference(frole*3000, rev::ControlType::kVelocity);
  
    pidfr.SetReference(frole*3000, rev::ControlType::kVelocity);
    
    pidbl.SetReference(bale*3000, rev::ControlType::kVelocity);
   
    pidbr.SetReference(bari*3000, rev::ControlType::kVelocity);

  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif