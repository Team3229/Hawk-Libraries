package frc.robot.hawklibaries.drivetrains.tank;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hawklibaries.vendorRewrites.rev.CANSparkMax;
import frc.robot.hawklibaries.vendorRewrites.rev.CANSparkMax.SetpointType;

/**
 * TankDrivetrain class represents a differential (tank) drivetrain for an FRC robot.
 * This subsystem controls a drivetrain with four motors (two per side) and uses a 
 * gyroscope (navX) for rotation data, a kinematic model for motion control, 
 * and an odometry system to track the robot's position on the field.
 */
public class TankDrivetrain extends SubsystemBase {

    // Configuration settings for the drivetrain, containing motor IDs, PID values, etc.
    private TankDrivetrainConfig config;

    // DifferentialDriveKinematics is responsible for the kinematic model of the drivetrain.
    private DifferentialDriveKinematics kinematics;

    // DifferentialDrivePoseEstimator helps with tracking the position (pose) of the robot.
    private DifferentialDrivePoseEstimator odometry;

    // Motor controllers for the left and right sides of the drivetrain
    private CANSparkMax leftLeadMotor;
    private CANSparkMax leftFollowerMotor;
    private CANSparkMax rightLeadMotor;
    private CANSparkMax rightFollowerMotor;

    // AHRS (Attitude and Heading Reference System) for retrieving the robot's rotation.
    private AHRS navX;

    // Field2d for visualizing the robot's position on the SmartDashboard
    private Field2d odometryField;

    // Current speed of the robot, represented as ChassisSpeeds
    private ChassisSpeeds robotCurrentSpeeds;

    /**
     * Constructs a new TankDrivetrain instance with specified configuration and default driving speeds.
     *
     * @param config The configuration object containing necessary settings for the drivetrain
     * @param defaultDrivingSpeeds A Supplier providing the default chassis speeds for the drivetrain. If unsure, use this to pass driver controller input.
     */
    public TankDrivetrain(
        TankDrivetrainConfig config,
        Supplier<ChassisSpeeds> defaultDrivingSpeeds
    ) {
        // Initialize configuration and motor controllers
        this.config = config;

        leftLeadMotor = new CANSparkMax(this.config.getLeftLeadID(), this.config.getMotorType());
        leftFollowerMotor = new CANSparkMax(this.config.getLeftFollowerID(), this.config.getMotorType());
        rightLeadMotor = new CANSparkMax(this.config.getRightLeadID(), this.config.getMotorType());
        rightFollowerMotor = new CANSparkMax(this.config.getRightFollowerID(), this.config.getMotorType());

        // Set PID constants for the lead motors
        leftLeadMotor.setPID(this.config.getSpeedPID());
        rightLeadMotor.setPID(this.config.getSpeedPID());

        // Set followers to match the lead motors
        leftFollowerMotor.follow(leftLeadMotor);
        rightFollowerMotor.follow(rightLeadMotor);

        // Initialize the gyroscope and kinematics/odometry systems
        navX = new AHRS();
        kinematics = new DifferentialDriveKinematics(config.getTrackWidth());
        odometry = new DifferentialDrivePoseEstimator(
            kinematics,
            navX.getRotation2d(),
            leftLeadMotor.getPosition(),
            rightLeadMotor.getPosition(),
            new Pose2d()
        );

        // Setup field visualization and default speeds
        odometryField = new Field2d();
        robotCurrentSpeeds = new ChassisSpeeds();

        // Set default command to drive using the provided speed supplier
        setDefaultCommand(driveCommand(defaultDrivingSpeeds));
    }

    /**
     * Generates a Command to drive the robot based on a supplier for ChassisSpeeds.
     *
     * @param speeds Supplier that provides the desired chassis speeds for the robot
     * @return Command that executes the drivetrain movement based on the supplier's speeds
     */
    private Command driveCommand(Supplier<ChassisSpeeds> speeds) {
        Command out = new Command() {
            @Override public void execute() {
                drive(speeds.get());
            }
        };

        // Add this subsystem as a requirement for the command
        out.addRequirements(this);
        return out;
    }

    /**
     * Drives the robot by setting motor velocities based on desired ChassisSpeeds.
     *
     * @param speeds The target chassis speeds for the robot in ChassisSpeeds format
     */
    public void drive(ChassisSpeeds speeds) {
        // Update current speeds and convert them to wheel speeds
        robotCurrentSpeeds = speeds;
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(robotCurrentSpeeds);
        
        // Ensure wheel speeds are within the maximum allowed range
        wheelSpeeds.desaturate(config.getMaxSpeed());

        // Set the wheel speeds to the motors using the kVelocity mode
        leftLeadMotor.setSetpoint(wheelSpeeds.leftMetersPerSecond, SetpointType.kVelocity);
        rightLeadMotor.setSetpoint(wheelSpeeds.rightMetersPerSecond, SetpointType.kVelocity);
    }

    @Override
    public void periodic() {
        // Update odometry with current sensor positions and gyro angle
        odometry.update(
            navX.getRotation2d(),
            leftLeadMotor.getPosition(),
            rightLeadMotor.getPosition()
        );

        // Optional: Update field visualization
        odometryField.setRobotPose(odometry.getEstimatedPosition());
    }

    /**
     * Updates SmartDashboard with key drivetrain metrics for diagnostics.
     */
    @Override
    public void initSendable(SendableBuilder builder) {

        super.initSendable(builder);

        SmartDashboard.putData("Field", odometryField); // Shows robot position on the field
    }


}