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
    //m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);

    // Set drive motors to brake mode
    //m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    //m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    //m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    //m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

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
  int driveMod = Drive_Mode % 4;
  frc::SmartDashboard::PutNumber("Drive Mode: ", driveMod);
  //0 Old Arcade, 1 Tank, 2 New Arcade
  /*
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
  */

  
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
  if (controller->GetYButtonPressed()) {
    Drive_Mode = Drive_Mode + 1;
  }
  if (Drive_Mode % 4 == 0) { // MyArcade
    double left_y = DzShift(controller->GetLeftY(), 0.02);
    double right_x = DzShift(controller->GetRightX(), 0.02);

    // Turning
    if (right_x > 0 || right_x < 0) {
      LL = right_x; 
      RL = -right_x;
    } else {
      LL = left_y;
      RL = left_y;
    }
  } else if (Drive_Mode % 4 == 1) { // My Tank
    double left_y = DzShift(controller->GetLeftY(), 0.2);
    double right_y = DzShift(controller->GetRightY(), 0.2);
    LL = left_y;
    RL = right_y;
  } else if (Drive_Mode % 4 == 2) { // My Arcade Trigger
    double Ltrigger = DzShift(controller->GetLeftTriggerAxis());
    double Rtrigger = DzShift(controller->GetRightTriggerAxis());
    double Rturn = DzShift(controller->GetLeftX());
    if (Ltrigger > 0) { // Brake or Reverse
      LL = -Ltrigger;
      RL = -Ltrigger;
    }
    else if (Rtrigger > 0) {
      if (!(Rturn == 0)) { // Turning while going forward
        if (Rturn > 0) { // Turning Right
          LL = Rtrigger;
          RL = Rtrigger - (Rturn * (1 - Rtrigger));
        } else if (Rturn < 0) {
          LL = Rtrigger - (fabs(Rturn) * (1-Rtrigger));
          RL = Rtrigger;
        }   
    } else { // Moving Straight forward
        LL = Rtrigger;
        RL = Rtrigger;
    } 
    
    } else {
        if (!(Rturn == 0)) { // Turning in place
          LL = Rturn;
          RL = -Rturn;
        } else { // No input
          LL = 0.0;
          RL = 0.0;
        }
  }

} else if (Drive_Mode % 4 == 3) { // FRC Arcade
  m_robotdrive->ArcadeDrive(DzShift(controller->GetLeftY()), DzShift(controller->GetRightX()));
  return;
}
  if (LL > 1) {
    LL == 1;
  }
  if (RL > 1) {
    RL = 1;
  }
  m_leftLeadMotor->Set(LL);
  m_rightLeadMotor->Set(RL);
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
