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
}

void Robot::AutonomousPeriodic() 
{
  ExecuteControls();
}

void Robot::TeleopInit() 
{
 
}

void Robot::TeleopPeriodic() 
{
  m_turret.turretDash();

  ExecuteControls();
}

void Robot::TestInit() 
{
  
}

void Robot::TestPeriodic() 
{
  
}

void Robot::DisabledInit()
{
 
}

// TeleOp
void Robot::ExecuteControls()
{
  
    // Aiming the shooter
    if (std::abs(m_controllerInputs->mani_rightX) > .1) {
      m_turret.Turn(m_controllerInputs->mani_rightX/5);
    } else {
      m_turret.Turn(0);
    }

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif 