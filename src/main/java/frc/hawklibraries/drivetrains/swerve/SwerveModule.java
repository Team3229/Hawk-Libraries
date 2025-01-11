/**
 * Class representing a swerve module, consisting of a drive motor, turning motor, and encoder for
 * positional feedback. This implementation provides methods for initializing and configuring the
 * module, as well as for setting desired states.
 */
package frc.hawklibraries.drivetrains.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A class for managing an individual swerve module in a drivetrain. This includes controlling the
 * drive and turning motors and providing feedback for optimization.
 */
public class SwerveModule {

  /** Configuration object containing module parameters. */
  SwerveModuleConfig config;

  /** Motor for driving the wheel. */
  SparkMax m_driveMotor;

  /** Motor for controlling the turning angle. */
  SparkMax m_turningMotor;

  SparkMaxConfig m_driveMotorConfig;

  SparkMaxConfig m_turningMotorConfig;

  /** Encoder for measuring the turning angle. */
  CANcoder m_turningEncoder;

  /** PID controller for the drive motor. */
  SparkClosedLoopController m_drivePIDController;

  /** PID controller for the turning motor. */
  SparkClosedLoopController m_turningPIDController;

  /**
   * Constructs a new SwerveModule instance.
   *
   * @param config Configuration object with parameters such as motor IDs, PID constants, and gear
   *     ratios.
   */
  public SwerveModule(SwerveModuleConfig config) {

    this.config = config;

    // Initialize motors and encoder with IDs from configuration
    m_driveMotor = new SparkMax(this.config.getDriveID(), MotorType.kBrushless);
    m_turningMotor = new SparkMax(this.config.getTurningID(), MotorType.kBrushless);
    m_turningEncoder = new CANcoder(this.config.getTurningEncoderID());
    m_drivePIDController = m_driveMotor.getClosedLoopController();
    m_turningPIDController = m_turningMotor.getClosedLoopController();

    // Set motor idle modes and configure position/velocity conversion factors
    m_driveMotorConfig.idleMode(IdleMode.kCoast);
    m_driveMotorConfig
        .encoder
        .positionConversionFactor(Math.PI * config.getWheelDiameter() / config.getDriveGearRatio())
        .velocityConversionFactor(
            Math.PI * config.getWheelDiameter() / 60 / config.getDriveGearRatio());

    m_turningMotorConfig.idleMode(IdleMode.kCoast).inverted(true);
    m_turningMotorConfig.encoder.positionConversionFactor(2 * Math.PI / config.getDriveGearRatio());
    m_turningMotorConfig
        .closedLoop
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .positionWrappingEnabled(true);

    // Apply encoder offset
    m_turningEncoder
        .getConfigurator()
        .setPosition(
            m_turningEncoder.getPosition().getValueAsDouble()
                + this.config.getEncoderOffset().getRotations());

    // Seed motor position based on encoder feedback
    seedSparkMax();
  }

  /**
   * Retrieves the current position of the swerve module.
   *
   * @return A SwerveModulePosition object containing the module's position.
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition();
  }

  /**
   * Sets the desired state for the swerve module.
   *
   * @param desiredState The desired state, including speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = new Rotation2d(m_turningMotor.getEncoder().getPosition());

    // Optimize the desired state to minimize unnecessary rotation

    desiredState.optimize(currentRotation);

    // Adjust speed for angle error
    desiredState.speedMetersPerSecond *= desiredState.angle.minus(currentRotation).getCos();

    // Apply speed and angle to respective controllers
    m_drivePIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
  }

  /** Aligns the turning motor's position with the encoder's measured position. */
  private void seedSparkMax() {
    m_turningMotor
        .getEncoder()
        .setPosition(m_turningEncoder.getPosition().getValueAsDouble() * 2 * Math.PI);
  }
}
