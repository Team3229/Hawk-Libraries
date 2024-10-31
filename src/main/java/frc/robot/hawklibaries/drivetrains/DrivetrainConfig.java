package frc.robot.hawklibaries.drivetrains;

import lombok.Getter;
import lombok.Setter;
import lombok.Builder.Default;
import lombok.experimental.SuperBuilder;

/**
 * Configuration class for drivetrain parameters.
 * Contains key parameters for drivetrain behavior including wheel size, speed, acceleration, 
 * turning rate, and robot dimensions. This allows flexible configuration for different drivetrain designs.
 */
@SuperBuilder
@Getter
@Setter
public class DrivetrainConfig {

    /**
     * Diameter of the wheels in meters.
     * <p><b>Default:</b> 0.25 meters (25 cm)
     */
    @Default
    private double wheelDiameter = 0.1016;

    /**
     * Maximum speed of the drivetrain in meters per second.
     * <p><b>Default:</b> 3.0 m/s, which is standard for FRC robot drivetrains.
     */
    @Default
    private double maxSpeed = 3.0;

    /**
     * Maximum acceleration of the drivetrain in meters per second squared.
     * <p><b>Default:</b> 1.0 m/s². This should prevent wheel slip and ensure manageable acceleration.
     */
    @Default
    private double maxAccel = 1.0;

    /**
     * Maximum turning rate of the drivetrain in radians per second.
     * <p><b>Default:</b> 1.0 rad/s. Modify based on testing if the turn rate feels too slow or too fast.
     */
    @Default
    private double maxTurnRate = 1.0;

    /**
     * Width of the robot in meters.
     * <p><b>Default:</b> 1.0 m. Adjust according to the robot's actual width.
     */
    @Default
    private double robotWidth = 1.0;

    /**
     * Length of the robot in meters.
     * <p><b>Default:</b> 1.0 m. Adjust according to the robot's actual length.
     */
    @Default
    private double robotLength = 1.0;
}