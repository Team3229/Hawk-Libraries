/**
 * Class representing a swerve module, consisting of a drive motor, turning motor,
 * and encoder for positional feedback. This implementation provides methods for
 * initializing and configuring the module, as well as for setting desired states.
 */
package frc.hawklibraries.drivetrains.swerve;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.hawklibraries.vendorRewrites.ctre.CANcoder;
import frc.hawklibraries.vendorRewrites.rev.CANSparkMax;
import frc.hawklibraries.vendorRewrites.rev.CANSparkMax.SetpointType;
import frc.hawklibraries.vendorRewrites.wpilib.PIDController;

/**
 * A class for managing an individual swerve module in a drivetrain.
 * This includes controlling the drive and turning motors and providing feedback for optimization.
 */
public class SwerveModule {

    /** Configuration object containing module parameters. */
    SwerveModuleConfig config;

    /** Motor for driving the wheel. */
    CANSparkMax m_driveMotor;

    /** Motor for controlling the turning angle. */
    CANSparkMax m_turningMotor;

    /** Encoder for measuring the turning angle. */
    CANcoder m_turningEncoder;

    /** PID controller for the drive motor. */
    PIDController m_drivePIDController;

    /** PID controller for the turning motor. */
    PIDController m_turningPIDController;
    
    /**
     * Constructs a new SwerveModule instance.
     *
     * @param config Configuration object with parameters such as motor IDs, PID constants, and gear ratios.
     */
    public SwerveModule(SwerveModuleConfig config) {

        this.config = config;

        // Initialize motors and encoder with IDs from configuration
        m_driveMotor = new CANSparkMax(this.config.getDriveID(), MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(this.config.getTurningID(), MotorType.kBrushless);
        m_turningEncoder = new CANcoder(this.config.getTurningEncoderID());
        m_drivePIDController = new PIDController(this.config.getDrivePID());
        m_turningPIDController = new PIDController(this.config.getTurningPID());

        // Reset motors to factory defaults
        m_driveMotor.restoreFactoryDefaults();
        m_turningMotor.restoreFactoryDefaults();

        // Set motor idle modes and configure position/velocity conversion factors
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turningMotor.setIdleMode(IdleMode.kCoast);
        m_driveMotor.setPositionConversionFactor(Math.PI * config.getWheelDiameter() / config.getDriveGearRatio());
        m_driveMotor.setVelocityConversionFactor(Math.PI * config.getWheelDiameter() / 60 / config.getDriveGearRatio());
        m_driveMotor.setPosition(0);
        m_turningMotor.setPositionConversionFactor(2 * Math.PI / config.getDriveGearRatio());
        m_turningMotor.setInverted(true);

        // Configure PID controllers
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_drivePIDController.applyToSparkMax(m_driveMotor, SetpointType.kVelocity);
        m_turningPIDController.applyToSparkMax(m_turningMotor, SetpointType.kPosition);

        // Apply encoder offset
        m_turningEncoder.setOffset(this.config.getEncoderOffset());

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
        Rotation2d currentRotation = new Rotation2d(m_turningMotor.getPosition());

        // Optimize the desired state to minimize unnecessary rotation
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentRotation);

        // Adjust speed for angle error
        optimizedState.speedMetersPerSecond *= optimizedState.angle.minus(currentRotation).getCos();

        // Apply speed and angle to respective controllers
        m_drivePIDController.setSetpoint(optimizedState.speedMetersPerSecond);
        m_turningPIDController.setSetpoint(optimizedState.angle.getRadians());
    }

    /**
     * Aligns the turning motor's position with the encoder's measured position.
     */
    private void seedSparkMax() {
        m_turningMotor.setPosition(m_turningEncoder.getPosition().getRadians());
    }
}
