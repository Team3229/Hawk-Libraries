/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

// Our needed includes
#include <frc/XboxController.h>
#include <Math.h>

// Subsystem includes
#include "DriveSystem.h"
#include "Debug.h"

class Robot : public frc::TimedRobot {
 public:
   void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
  void DisabledInit() override;

 private:
  // SmartDashboard setup
  frc::SendableChooser<std::string> m_chooser;
  std::string m_driveSelected;

  // Constants
  const int XBOX_USB_DRIVER_1 = 0;
  const float DEAD_BAND = 0.1;

  // Controller variables
  double d1_leftY, d1_leftX, d1_rightX, d2_leftY, d2_rightY;
  int m_lastUsedSpeed = 2;
  

  // Objects of subsystems
  DriveSystem chassis{};


  
  frc::XboxController xbox1 {XBOX_USB_DRIVER_1}; //Chassis driver

};