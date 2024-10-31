package frc.robot.hawklibaries.drivetrains.tank;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.hawklibaries.drivetrains.DrivetrainConfig;
import frc.robot.hawklibaries.utilities.PIDConstants;

import lombok.Getter;
import lombok.Setter;
import lombok.Builder.Default;
import lombok.experimental.SuperBuilder;

/**
 * Configuration class for a tank drivetrain setup, extending base drivetrain configuration.
 * This class defines specific parameters for tank drivetrain, including motor IDs,
 * motor type, track width, and PID settings for speed control.
 */
@SuperBuilder
@Getter
@Setter
public class TankDrivetrainConfig extends DrivetrainConfig {

    /** CAN ID for the left lead motor controller. */
    int leftLeadID;

    /** CAN ID for the left follower motor controller. */
    int leftFollowerID;

    /** CAN ID for the right lead motor controller. */
    int rightLeadID;

    /** CAN ID for the right follower motor controller. */
    int rightFollowerID;

    /** 
     * The motor type for the drivetrain motors
     * <p><b>Default:</b> kBrushless
    */
    @Default
    MotorType motorType = MotorType.kBrushless;

    /** The track width (distance between the left and right wheels) of the drivetrain in meters. */
    double trackWidth;

    /** PID constants for controlling the speed of the drivetrain. */
    PIDConstants speedPID;
}
