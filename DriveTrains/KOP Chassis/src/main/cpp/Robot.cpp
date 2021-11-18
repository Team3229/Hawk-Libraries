/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() 
{
 
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
  chassis.ResetGyro();
  chassis.ChangeSpeed(3); // turbo speed
}

void Robot::AutonomousPeriodic() 
{
  TeleopPeriodic();
}

void Robot::TeleopInit() 
{

}

void Robot::TeleopPeriodic()
{    
  //Update controller axis values
  d1_leftY = xbox1.GetY(frc::GenericHID::kLeftHand);
  d1_leftX = xbox1.GetX(frc::GenericHID::kLeftHand);
  d1_rightX = xbox1.GetX(frc::GenericHID::kRightHand);

		if(abs(d1_leftX) > DEAD_BAND || abs(d1_leftY) > DEAD_BAND)
		{
			  chassis.Drive(d1_leftY, d1_leftX, d1_rightX);
		}
		else
		{
			  chassis.Stop();
  
  // speed changer 
  // BOTH CONTROLLERS NOW HAVE ACCESS TO THESE
  if (xbox1.GetAButton())
  {
    chassis.ChangeSpeed(2); // normal speed
    m_lastUsedSpeed = 2;
  }

  if (xbox1.GetBButton())
  {
    chassis.ChangeSpeed(1); // slow speed
    m_lastUsedSpeed = 1;
  }

  if (xbox1.GetXButton())
  {
    chassis.ChangeSpeed(3); // fast
    m_lastUsedSpeed = 3;
  }

}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif 




