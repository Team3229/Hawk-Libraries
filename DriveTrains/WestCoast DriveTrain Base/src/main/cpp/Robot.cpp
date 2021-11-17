/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
  m_auto.SetupPlayback();
}

void Robot::AutonomousPeriodic() 
{
  m_auto.ReadFile(m_controllerInputs);
  ExecuteControls();
}

void Robot::TeleopInit() 
{
  m_auto.CloseFile();
}

void Robot::TeleopPeriodic() 

  m_drive.drivetrainDash();
  m_turret.turretDash();

  ExecuteControls();
}

void Robot::TestInit() 
{

}

void Robot::TestPeriodic() 
{
  if (m_recordMode) { // recording
    // Run TeleOp as normal
    
  }
}

void Robot::DisabledInit()
{
  m_auto.CloseFile();
}

// TeleOp
void Robot::ExecuteControls()
{
  // Drive - ADD POWER CURVE
  if (kDRIVEDEADBAND > std::abs(m_controllerInputs->drive_rightY) && 
      kDRIVEDEADBAND > std::abs(m_controllerInputs->drive_leftX)) {
    m_drive.StopMotor();
  } else if(m_slowDriveMode){
    m_drive.Drive(m_controllerInputs->drive_rightY*m_drive.kSlowMaxSpeed,
                  -m_controllerInputs->drive_leftX*m_drive.kMaxAngularSpeed);
  } else {
    m_drive.Drive(m_controllerInputs->drive_rightY*m_drive.kMaxSpeed,
                  -m_controllerInputs->drive_leftX*m_drive.kMaxAngularSpeed);
  }
  m_drive.UpdateOdometry();
  //slow mode - 4mps (affects acceleration and fine control)
  if(m_controllerInputs->drive_AButton){
    m_slowDriveMode = true;
  }
  //fast mode - 8 mps (affects acceleration and fine control)
  if(m_controllerInputs->drive_BButton){
    m_slowDriveMode = false;
  }
  
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif 