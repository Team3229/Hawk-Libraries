//Author: Sebastian Roman (Programming Lead)
//Co Author: 3229 Programming

#include "DriveSystem.h"

DriveSystem::DriveSystem()
{
    //Instantiate motor controllers
	leftLead = new WPI_TalonSRX(LEFT_LEAD_ID);
	rightLead = new WPI_TalonSRX(RIGHT_LEAD_ID);
	leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
	rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

	 //Reset the controllers
	leftLead->Set(ControlMode::PercentOutput, 0);
	rightLead->Set(ControlMode::PercentOutput, 0);
	leftFollower->Set(ControlMode::PercentOutput, 0);
	rightFollower->Set(ControlMode::PercentOutput, 0);

		//Sets smoothing curve to talons
	leftLead->ConfigOpenloopRamp(SMOOTH_TIME, 0); //passes in seconds from neutral to full and timeout in miliseconds
	rightLead->ConfigOpenloopRamp(SMOOTH_TIME, 0);
	leftFollower->ConfigOpenloopRamp(SMOOTH_TIME, 0);
	rightFollower->ConfigOpenloopRamp(SMOOTH_TIME, 0);

	//Clears sticky faults
	leftLead->ClearStickyFaults(0);
	rightLead->ClearStickyFaults(0);
	leftFollower->ClearStickyFaults(0);
	rightFollower->ClearStickyFaults(0);

    //Set followers
	leftFollower->Follow(*leftLead);
	rightFollower->Follow(*rightLead);

	//Instantiate DriveTrain
	driveTrain = new frc::DifferentialDrive(*leftLead, *rightLead);

	//Sets max initial speed and turn off safety
	driveTrain->SetMaxOutput(MAX_OUTPUT);
	driveTrain->SetSafetyEnabled(false); 
	driveTrain->SetExpiration(SAFETY_TIMEOUT); //Set safety timer

}

DriveSystem::~DriveSystem()
{
    delete leftLead;
	delete rightLead;
	delete leftFollower;
	delete rightFollower;
	delete driveTrain;

}

void DriveSystem::Drive(double& Y, double& X, double& Z)
{
    //Drive function
	Y = -Y; // invert Y
    driveTrain->ArcadeDrive(Y, X); //might cause issues due to update? needs testing
	debug("Drive mode: With Gyro\n");
}

void DriveSystem::Stop()
{
    driveTrain->StopMotor();
}

void DriveSystem::ChangeSpeed(int choice)
{
    if (choice == 1)
	{
        driveTrain->SetMaxOutput(LOW_OUTPUT);
    	debug("Slow speed\n");
	}
    else if (choice == 2)
	{
        driveTrain->SetMaxOutput(MAX_OUTPUT);
    	debug("Normal speed\n");
	}
    else if (choice == 3)
	{
        driveTrain->SetMaxOutput(HIGH_OUTPUT);
    	debug("Fast speed\n"); 
	}
}
