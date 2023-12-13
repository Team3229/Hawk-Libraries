
#include "Robot.h"
#include <iostream>

void Robot::RobotInit(){}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
  m_auto.SetupPlayback();
   
}

void Robot::AutonomousPeriodic() 
{
  m_auto.ReadFile(m_controllerInputs);
  debugCons("Running Auto");
  ExecuteControls();
}

void Robot::TeleopInit() 
{
  m_auto.CloseFile();
}

void Robot::TeleopPeriodic()
{    
  // Populate controller struct
  m_controllerInputs->driver_rightY = xbox1.GetY(frc::GenericHID::kRightHand);
  m_controllerInputs->driver_rightX = xbox1.GetX(frc::GenericHID::kRightHand);
  m_controllerInputs->driver_leftX = xbox1.GetX(frc::GenericHID::kLeftHand);
  m_controllerInputs->drive_AButton = xbox1.GetAButton();
  m_controllerInputs->drive_BButton = xbox1.GetBButton();
  m_controllerInputs->drive_XButton = xbox1.GetXButton();
  m_controllerInputs->drive_YButton = xbox1.GetYButton();
  m_controllerInputs->drive_RightBumper = xbox1.GetBumper(frc::GenericHID::kRightHand);
  m_controllerInputs->drive_LeftBumper = xbox1.GetBumper(frc::GenericHID::kLeftHand);
  m_controllerInputs->drive_RightTriggerAxis = xbox1.GetTriggerAxis(frc::GenericHID::kRightHand);
  m_controllerInputs->drive_LeftTriggerAxis = xbox1.GetTriggerAxis(frc::GenericHID::kLeftHand);
  m_controllerInputs->mani_rightY = xbox2.GetY(frc::GenericHID::kRightHand);
  m_controllerInputs->mani_rightX = xbox2.GetX(frc::GenericHID::kRightHand);
  m_controllerInputs->mani_leftY = xbox2.GetY(frc::GenericHID::kLeftHand);
  m_controllerInputs->mani_leftX = xbox2.GetX(frc::GenericHID::kLeftHand);
  m_controllerInputs->mani_AButton = xbox2.GetAButton();
  m_controllerInputs->mani_BButton = xbox2.GetBButton();
  m_controllerInputs->mani_XButton = xbox2.GetXButton();
  m_controllerInputs->mani_YButton = xbox2.GetYButton();
  m_controllerInputs->mani_RightBumper = xbox2.GetBumper(frc::GenericHID::kRightHand);
  m_controllerInputs->mani_LeftBumper =  xbox2.GetBumper(frc::GenericHID::kLeftHand);
  m_controllerInputs->mani_RightTriggerAxis =  xbox2.GetTriggerAxis(frc::GenericHID::kRightHand);
  m_controllerInputs->mani_LeftTriggerAxis =  xbox2.GetTriggerAxis(frc::GenericHID::kLeftHand);

    ExecuteControls();
}

void Robot::TestInit() 
{
  m_auto.SetupRecording();
}

void Robot::TestPeriodic() 
{
  if (m_recordMode) { // recording
    // Run TeleOp as normal
    TeleopPeriodic();
    // Write current struct to file
    m_auto.Record(m_controllerInputs);
  }
}

void Robot::DisabledInit() 
{
  m_auto.CloseFile();
}

void Robot::ExecuteControls()
{
if(abs(DEAD_BAND > std::abs(m_controllerInputs->driver_rightY) && DEAD_BAND > std::abs(m_controllerInputs->driver_leftX)))
		{
        chassis.Stop();
		}
		else
		{
			 chassis.Drive(m_controllerInputs->driver_rightY, m_controllerInputs->driver_rightX, m_controllerInputs->driver_leftX);
    }
  
  // speed changer 
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


#ifndef RUNNING_FRC_TESTS

int main() { return frc::StartRobot<Robot>(); }
#endif 




