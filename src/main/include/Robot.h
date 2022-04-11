// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SendableChooser.h>

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
  frc::Joystick* m_stick = new frc::Joystick{0};
  frc::XboxController* controller = new frc::XboxController{0};

  //Motors
  static const int leftLeadDeviceID = 12; 
  static const int leftFollowDeviceID = 13; 
  static const int rightLeadDeviceID = 15; 
  static const int rightFollowDeviceID = 14; 
  const int driveMotorCurrentLimit = 30;
  rev::CANSparkMax* m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  int Drive_Mode = 0; // 0 Arcade, 1 Tank, 2 Forza, 3 Swerve(eventually)
  //Functions
  double DzShift(double input, double dz);

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};

double Robot::DzShift(double input, double dz) {
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