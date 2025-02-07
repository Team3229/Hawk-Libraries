/**
 * Configuration class for a swerve module, which includes parameters and settings for both the
 * drive and turning mechanisms. This class uses Lombok annotations to reduce boilerplate code and
 * provides defaults for certain parameters.
 */
package frc.hawklibraries.drivetrains.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.hawklibraries.utilities.PIDConstants;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.SuperBuilder;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * Configuration for an individual swerve module. <br>
 * This class encapsulates all the necessary parameters to configure a swerve module, including gear
 * ratios, motor IDs, encoder offsets, and PID constants for driving and turning.
 */
@SuperBuilder
@Getter
@Setter
public class SwerveModuleConfig {

  /**
   * Diameter of the swerve module wheel, in meters. <br>
   * <b>Default</b>: 0.25 meters (25 cm).
   */
  @Builder.Default private Distance wheelDiameter = Meter.of(0.25);

  /**
   * Gear ratio for the drive motor, represented as the ratio of motor rotations to wheel rotations.
   * <br>
   * <b>Default</b>: 1.0 (1:1 ratio).
   */
  @Builder.Default private double driveGearRatio = 1.0;

  /**
   * Gear ratio for the turning motor, represented as the ratio of motor rotations to turning
   * mechanism rotations. <br>
   * <b>Default</b>: 1.0 (1:1 ratio).
   */
  @Builder.Default private double turningGearRatio = 1.0;

  /**
   * Maximum speed of the drive motor, in meters per second. <br>
   * <b>Default</b>: 3.0 meters/second.
   */
  @Builder.Default private LinearVelocity maxSpeed = MetersPerSecond.of(3.0);

  /**
   * Encoder offset for the turning motor, represented as a Rotation2d object. Used to correct the
   * initial position of the turning mechanism. <br>
   * <b>Default</b>: 0 radians.
   */
  @Builder.Default private Rotation2d encoderOffset = new Rotation2d(0);

  /** CAN ID of the drive motor controller. */
  private int driveID;

  /** CAN ID of the turning motor controller. */
  private int turningID;

  /** CAN ID of the turning encoder. */
  private int turningEncoderID;

  /**
   * PID constants for controlling the drive motor. These constants define the behavior of the PID
   * controller for driving.
   */
  private PIDConstants drivePID;

  /**
   * PID constants for controlling the turning motor. These constants define the behavior of the PID
   * controller for turning.
   */
  private PIDConstants turningPID;

  /**
   * Minimum output for the turning motor, typically used to limit the range of the PID controller.
   */
  private double turningOutputMin;

  /**
   * Maximum output for the turning motor, typically used to limit the range of the PID controller.
   */
  private double turningOutputMax;

  /**
   * Inverts the direction of the drive motor. <br>
   * <b>Default</b>: false.
   */
  private boolean invertDriveMotor;
}
