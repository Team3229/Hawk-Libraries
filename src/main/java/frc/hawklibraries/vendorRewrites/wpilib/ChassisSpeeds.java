package frc.hawklibraries.vendorRewrites.wpilib;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents the speeds of a robot chassis in meters per second and radians per second.
 * This class provides methods to convert between field-relative and robot-relative speeds.
 */
public class ChassisSpeeds {

  /**
   * The velocity in the x direction (forward).
   */
  public Double vxMetersPerSecond;

  /**
   * The velocity in the y direction (sideways).
   */
  public Double vyMetersPerSecond;

  /**
   * The angular velocity around the z axis (rotation).
   */
  public Double omegaRadiansPerSecond;

  /**
   * Default constructor for ChassisSpeeds.
   */
  public ChassisSpeeds() {}

  /**
   * Constructs a ChassisSpeeds object from an existing WPILib ChassisSpeeds object.
   *
   * @param speeds WPILib ChassisSpeeds object.
   */
  public ChassisSpeeds(edu.wpi.first.math.kinematics.ChassisSpeeds speeds) {
    this.vxMetersPerSecond = speeds.vxMetersPerSecond;
    this.vyMetersPerSecond = speeds.vyMetersPerSecond;
    this.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
  }

  /**
   * Constructs a ChassisSpeeds object with specified speeds.
   *
   * @param vxMetersPerSecond Speed in the x direction (forward) in meters per second.
   * @param vyMetersPerSecond Speed in the y direction (sideways) in meters per second.
   * @param omegaRadiansPerSecond Angular speed in radians per second.
   */
  public ChassisSpeeds(
      Double vxMetersPerSecond, Double vyMetersPerSecond, Double omegaRadiansPerSecond) {
    this.vxMetersPerSecond = vxMetersPerSecond;
    this.vyMetersPerSecond = vyMetersPerSecond;
    this.omegaRadiansPerSecond = omegaRadiansPerSecond;
  }

  /**
   * Converts field-relative speeds to robot-relative speeds.
   *
   * @param fieldRelativeSpeeds Field-relative speeds.
   * @param robotAngle Current angle of the robot.
   * @return Robot-relative speeds.
   */
  public static ChassisSpeeds fromFieldRelativeSpeeds(
      ChassisSpeeds fieldRelativeSpeeds, Rotation2d robotAngle) {
    return new ChassisSpeeds(
        edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds.getChassisSpeeds(), robotAngle));
  }

  /**
   * Converts this object to a WPILib ChassisSpeeds object.
   *
   * @return WPILib ChassisSpeeds object.
   */
  public edu.wpi.first.math.kinematics.ChassisSpeeds getChassisSpeeds() {
    return new edu.wpi.first.math.kinematics.ChassisSpeeds(
        vxMetersPerSecond == null ? 0 : vxMetersPerSecond,
        vyMetersPerSecond == null ? 0 : vyMetersPerSecond,
        omegaRadiansPerSecond == null ? 0 : omegaRadiansPerSecond);
  }

  /**
   * Discretizes continuous speeds for a given time step.
   *
   * @param continuousSpeeds Continuous speeds.
   * @param dtSeconds Time step in seconds.
   * @return Discretized speeds.
   */
  public static ChassisSpeeds discretize(ChassisSpeeds continuousSpeeds, double dtSeconds) {
    return new ChassisSpeeds(
        edu.wpi.first.math.kinematics.ChassisSpeeds.discretize(
            continuousSpeeds.getChassisSpeeds(), dtSeconds));
  }
}
