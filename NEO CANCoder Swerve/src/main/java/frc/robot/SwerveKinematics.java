package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;

public class SwerveKinematics {

    // Swerve Modules
    private static SwerveModule frontLeftModule;
    private static SwerveModule frontRightModule;
    private static SwerveModule backLeftModule;
    private static SwerveModule backRightModule;

    /**Array of the angle offsets for each swerve module. (Edit the values here to your needs)*/
    private static final Rotation2d[] moduleOffsets = {Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)};

    /**PID values for the swerve modules' angular motion. (Automatically populated with our constants we used for the 22-23 season) */
    private static final double[] anglePID = {0.01, 0.0001, 0};
    /**PID values for the swerve modules' driving motion. (Automatically populated with our constants we used for the 22-23 season) */
    private static final double[] drivePID = {0.1, 0, 0};

    /**Kauai Labs NavX Gyro. */
    public static AHRS navxGyro;
    /**The current rotation of the chassis. */
    public static Rotation2d robotRotation = Rotation2d.fromDegrees(0);
    /**
     * Robot relative mode uses the current rotation of the robot as forward, instead of the default
     * field-relative mode. This is useful for picking up objects or lining up with a game piece.
     */
    private static boolean relativeMode;

    /**An object used to calculate module velocities from overall chassis movement. */
    private static SwerveDriveKinematics kinematics;
    /**The current state of chassis velocity. */
    private static ChassisSpeeds chassisState;
    /**The current set module states. */
    private static SwerveModuleState[] moduleStates;

    /**The width of the robot chassis in meters. */
    private static final double robotWidth = 0.762;
    /**The maximum speed (in meters/sec) that a singular swerve module can reach. */
    private static final double maxModuleSpeed = 12;
    /**The maximum linear speed (in meters/sec) the chassis should move at. (Automatically set for SDS MK4 L1 modules) */
    private static final double chassisSpeed = 3.6576;
    /**The maximum angular speed (in radians/sec) that the chassis can rotate at. */
    private static final double maxChassisRotationSpeed = 0.75;
    /**Whether or not to run the drive motors in brake mode. */
    private static final boolean brakeMode = true;
    
    public SwerveKinematics() {}

    /**Initializes the drivetrain. */
    public static void initialize() {

        // Replace the CAN IDs to suit your needs.
        frontLeftModule = new SwerveModule(1, 2, 9, new Translation2d(-(robotWidth/2), (robotWidth/2)));
        frontRightModule = new SwerveModule(6, 5, 10, new Translation2d((robotWidth/2), (robotWidth/2)));
        backLeftModule = new SwerveModule(4, 3, 11, new Translation2d(-(robotWidth/2), -(robotWidth/2)));
        backRightModule = new SwerveModule(8, 7, 12, new Translation2d((robotWidth/2), -(robotWidth/2)));

        navxGyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(frontLeftModule.location, frontRightModule.location, backLeftModule.location, backRightModule.location);

        configureEncoders();
        configureMotors();
        configurePID();
        navxGyro.calibrate();
    }

    /**
     * Drives the chassis given an x, y, and rotational movement speed.
     * @param X X axis speed (left/right relative to driverstation)
     * @param Y Y axis speed (forward/backward relative to driverstation)
     * @param Z Rotational speed
     */
    public static void drive(double X, double Y, double Z) {

        // If we are in robot relative mode, override the chassis rotation to 180 degrees.
        if (relativeMode) {
            robotRotation = Rotation2d.fromDegrees(180);
        } else {
            robotRotation = Rotation2d.fromDegrees(navxGyro.getYaw());
        }

        // Calculate reverse kinematics
        chassisState = ChassisSpeeds.fromFieldRelativeSpeeds(Y*chassisSpeed, X*chassisSpeed, Z*maxChassisRotationSpeed, robotRotation);
        moduleStates = kinematics.toSwerveModuleStates(chassisState);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxModuleSpeed);

        // Set each module state
        frontLeftModule.setModuleState(moduleStates[0]);
        frontRightModule.setModuleState(moduleStates[1]);
        backLeftModule.setModuleState(moduleStates[2]);
        backRightModule.setModuleState(moduleStates[3]);
    }

    public static void zeroGyro() {
        navxGyro.zeroYaw();
    }

    /**Configures each module's motors. */
    public static void configureMotors() {

        frontLeftModule.configureAngleMotor();
        frontLeftModule.configureDriveMotor(brakeMode, false);
        frontRightModule.configureAngleMotor();
        frontRightModule.configureDriveMotor(brakeMode, false);
        backLeftModule.configureAngleMotor();
        backLeftModule.configureDriveMotor(brakeMode, false);
        backRightModule.configureAngleMotor();
        backRightModule.configureDriveMotor(brakeMode, true);
    }

    /**Configures and resets the offsets of each module's encoders. */
    public static void configureEncoders() {

        frontLeftModule.configureEncoders(moduleOffsets[0]);
        frontRightModule.configureEncoders(moduleOffsets[1]);
        backLeftModule.configureEncoders(moduleOffsets[2]);
        backRightModule.configureEncoders(moduleOffsets[3]);
    }

    /**Configures each module's PID Controllers with the provided constants at the top of this class. */
    public static void configurePID() {

        frontLeftModule.configurePID(anglePID, drivePID);
        frontRightModule.configurePID(anglePID, drivePID);
        backLeftModule.configurePID(anglePID, drivePID);
        backRightModule.configurePID(anglePID, drivePID);
    }

    /**Stops every module's motors. (should not be called during driving; the motors should continue to their setpoints) */
    public static void stop() {

        frontLeftModule.stopMotors();
        frontRightModule.stopMotors();
        backLeftModule.stopMotors();
        backRightModule.stopMotors();
    }

}