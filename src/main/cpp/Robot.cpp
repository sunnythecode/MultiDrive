// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    m_leftLeadMotor->RestoreFactoryDefaults();
    m_rightLeadMotor->RestoreFactoryDefaults();
    m_leftFollowMotor->RestoreFactoryDefaults();
    m_rightFollowMotor->RestoreFactoryDefaults();

    // Set current limit for drive motors
    m_leftLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
    m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
    m_leftFollowMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
    m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);

    // Set drive motors to brake mode
    m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Set followers and inverts for drive motors
    m_leftLeadMotor->SetInverted(true);
    m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
    m_rightLeadMotor->SetInverted(false);
    m_rightFollowMotor->Follow(*m_rightLeadMotor, false);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y", controller->GetLeftY());
  frc::SmartDashboard::PutNumber("right x", controller->GetRightX());
  
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  int left_y = DzShift(controller->GetLeftY(), 0.2);
  int right_x = DzShift(controller->GetRightX(), 0.2);
  //int j_ly = jstick->GetRawAxis(1);
  //int j_rx = jstick->GetRawAxis(4);
  double left_Final;
  double right_Final;
  // Turning
  if (right_x > 0.2 || right_x < -0.2) {
    left_Final = right_x; // add gradual change from 0 -> 0.2
    right_Final = -right_x;
} else {
    left_Final = left_y;
    right_Final = left_y;
}


  m_leftLeadMotor->Set(left_Final);
  m_rightLeadMotor->Set(right_Final);
  frc::SmartDashboard::PutNumber("Left speed:  ", left_Final);
  frc::SmartDashboard::PutNumber("Right speed:  ", right_Final);

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
