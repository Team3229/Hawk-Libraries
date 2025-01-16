package frc.hawklibraries.drivetrains.swerve;

import frc.hawklibraries.drivetrains.DrivetrainConfig;
import frc.hawklibraries.utilities.PIDConstants;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.SuperBuilder;

/** Configuration class for a swerve drivetrain. */
@SuperBuilder
@Getter
@Setter
public class SwerveDrivetrainConfig extends DrivetrainConfig {

  /** The distance between the swerve modules. */
  private double moduleDistance;

  /** PID constants for translation control. */
  private PIDConstants translationPID;

  /** PID constants for rotation control. */
  private PIDConstants rotationPID;

  /** Configuration for the front left swerve module. */
  private SwerveModuleConfig frontLeftConfig;

  /** Configuration for the front right swerve module. */
  private SwerveModuleConfig frontRightConfig;

  /** Configuration for the back left swerve module. */
  private SwerveModuleConfig backLeftConfig;

  /** Configuration for the back right swerve module. */
  private SwerveModuleConfig backRightConfig;
}
