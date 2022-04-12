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
    //m_leftLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
    //m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
    //m_leftFollowMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
    //m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit)    follower->Follow(*motor, false);
    // Set drive motors to brake mode
    m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Set followers and inverts for drive motors
    m_leftLeadMotor->SetInverted(true);
    m_leftFollowMotor->Follow(*m_leftLeadMotor, true);
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
  frc::SmartDashboard::PutNumber("Drive Mode: ", Drive_Mode);
  if ((Drive_Mode % 3) == 0) {
    frc::SmartDashboard::PutNumber("Raw Left y", controller->GetLeftY());
    frc::SmartDashboard::PutNumber("Dz Left y", DzShift(controller->GetLeftY(), 0.2));
    frc::SmartDashboard::PutNumber("Raw Right x", controller->GetRightX());
    frc::SmartDashboard::PutNumber("Dz Right x", DzShift(controller->GetRightX(), 0.2));
  } else if ((Drive_Mode % 3) == 1) {
    frc::SmartDashboard::PutNumber("Raw Left y", controller->GetLeftY());
    frc::SmartDashboard::PutNumber("Dz Left y", DzShift(controller->GetLeftY(), 0.2));
    frc::SmartDashboard::PutNumber("Raw Right y", controller->GetRightY());
    frc::SmartDashboard::PutNumber("Dz Right y", DzShift(controller->GetRightY(), 0.2));

  } else if ((Drive_Mode % 3) == 2) {
    frc::SmartDashboard::PutNumber("Right Trigger", controller->GetRightTriggerAxis());
    frc::SmartDashboard::PutNumber("Left Trigger", controller->GetRightTriggerAxis());
    frc::SmartDashboard::PutNumber("Dz Left x", DzShift(controller->GetLeftX(), 0.2));
  }

  
}

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
  double LL;
  double RL;
  double LF;
  double RF;
  if (controller->GetBButtonPressed()) {
    Drive_Mode = Drive_Mode + 1;
  }
  if (Drive_Mode % 3 == 0) {
    double left_y = DzShift(controller->GetLeftY(), 0.2);
    double right_x = DzShift(controller->GetRightX(), 0.2);

    // Turning
    if (right_x > 0.2 || right_x < -0.2) {
      LL = right_x; 
      RL = -right_x;
    } else {
      LL = left_y;
      RL = left_y;
    }
    RF = RL;
    LF = LL;
  } else if (Drive_Mode % 3 == 1) {
    double left_y = DzShift(controller->GetLeftY(), 0.2);
    double right_y = DzShift(controller->GetRightY(), 0.2);
    LL = left_y;
    RL = right_y;
    LF = LL;
    RF = RL;
  } else if (Drive_Mode % 3 == 2) {
    double brake = controller->GetLeftTriggerAxis();
    if (brake > 0.1) {
      LL = -brake;
      RL = -brake;
      LF = -brake;
      RF = -brake;
      return;
    }
    double speed = controller->GetRightTriggerAxis(); // speed for trigger
    double left_x = DzShift(controller->GetLeftX(), 0.2);
    if (left_x > 0) {
      RL = speed - left_x;
      LL = speed;
      RF = speed - left_x;
      LF = speed;
    } else if (left_x < 0) {
      RL = speed;
      LL = speed - left_x;
      RF = speed;
      LF = speed - left_x;
    } else if (left_x = 0) {
      RL = speed;
      LL = speed;
      RF = speed;
      LF = speed;
    }
    
  }


  m_leftLeadMotor->Set(LL);
  m_rightLeadMotor->Set(RL);
  m_leftFollowMotor->Set(LF);
  m_rightFollowMotor->Set(RF);

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
