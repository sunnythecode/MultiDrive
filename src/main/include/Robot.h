// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>


#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>



class Robot : public frc::TimedRobot {
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;


  //Controls
  //frc::Joystick* m_stick = new frc::Joystick{0};
  frc::XboxController* controller = new frc::XboxController{0};

/*
Uncommitted Branch:
#define lMotorLeaderID 15
#define lMotorFollowerID 16
#define rMotorLeaderID 9
#define rMotorFollowerID 3

Davis Branch:
#define lMotorLeaderID 15
#define lMotorFollowerID 11
#define rMotorLeaderID 9
#define rMotorFollowerID 13

*/
  //Motors
  static const int leftLeadDeviceID = 15; 
  static const int leftFollowDeviceID = 16; 
  static const int rightLeadDeviceID = 9; 
  static const int rightFollowDeviceID = 3; 
  const int driveMotorCurrentLimit = 60;
  rev::CANSparkMax* m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  frc::DifferentialDrive* m_robotdrive = new frc::DifferentialDrive(*m_leftLeadMotor, *m_rightLeadMotor);
  rev::SparkMaxRelativeEncoder lEncoder = m_leftLeadMotor->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder = m_rightLeadMotor->GetEncoder();
  int Drive_Mode = 0; // 0 Arcade, 1 Tank, 2 Forza, FRC Arcade
  //Functions
  double DzShift(double input, double dz);



 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};

double Robot::DzShift(double input, double dz=0.02) {
    if (dz > 0.02) {
        dz = 0.02;
    }
    double speed;
    if (fabs(input) < dz) {
        return 0.0;
    }
    if (input < 0) {
        double m = (1/(1-dz));
        double out = (m*(input-1))+1;
        return (out * out);
    } else {
        double m = (1/(1-dz));
        double out = (m*(fabs(input)-1))+1;
        return -(out * out);
    }
}