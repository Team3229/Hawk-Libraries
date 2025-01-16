/**
 * SwerveDrivetrain.java
 *
 * <p>Represents a swerve drive drivetrain subsystem in an FRC robot. This class integrates swerve
 * modules, odometry, kinematics, and autonomous capabilities to provide high-level control and
 * functionality.
 */
package frc.hawklibraries.drivetrains.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.hawklibraries.utilities.Alliance;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.hawklibraries.vendorRewrites.wpilib.ChassisSpeeds;
import java.util.function.Supplier;

/** Represents a swerve drivetrain subsystem. */
public class SwerveDrivetrain extends SubsystemBase {

  // Configuration object holding drivetrain-specific settings
  private SwerveDrivetrainConfig config;

  // Essential drivetrain components
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator odometry;
  private AHRS gyro;

  // Individual swerve modules
  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule backRight;

  // Field visualization and odometry
  private Field2d odometryField;
  private ChassisSpeeds robotCurrentSpeeds;

  // Autonomous command chooser
  private SendableChooser<Command> autoDropdown;

  // Driving overrides and speed input source
  private ChassisSpeeds autoOverrides;
  private Supplier<ChassisSpeeds> inputSpeeds;

  /**
   * Constructs a SwerveDrivetrain subsystem.
   *
   * @param config Configuration object containing drivetrain settings.
   * @param defaultDrivingSpeeds Supplier for default chassis speeds during teleoperation.
   */
  public SwerveDrivetrain(
      SwerveDrivetrainConfig config, Supplier<ChassisSpeeds> defaultDrivingSpeeds) {
    this.config = config;

    this.odometryField = new Field2d();

    // Initialize swerve modules
    this.frontLeft = new SwerveModule(this.config.getFrontLeftConfig());
    this.frontRight = new SwerveModule(this.config.getFrontRightConfig());
    this.backLeft = new SwerveModule(this.config.getBackLeftConfig());
    this.backRight = new SwerveModule(this.config.getBackRightConfig());

    // Initialize gyro for orientation tracking
    this.gyro = new AHRS(NavXComType.kMXP_SPI);

    // Configure drivetrain kinematics
    this.kinematics =
        new SwerveDriveKinematics(
            new Translation2d(this.config.getModuleDistance(), this.config.getModuleDistance()),
            new Translation2d(this.config.getModuleDistance(), -this.config.getModuleDistance()),
            new Translation2d(-this.config.getModuleDistance(), this.config.getModuleDistance()),
            new Translation2d(-this.config.getModuleDistance(), -this.config.getModuleDistance()));

    // Set up odometry for position tracking
    this.odometry =
        new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
              this.frontLeft.getModulePosition(),
              this.frontRight.getModulePosition(),
              this.backLeft.getModulePosition(),
              this.backRight.getModulePosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.7, 0.7, 9999999));

    // Adjust gyro alignment based on alliance color
    Alliance.alliancePresentTrigger()
        .onTrue(
            Commands.runOnce(
                () ->
                    gyro.setAngleAdjustment(
                        (Alliance.getAlliance() == AllianceColor.Red) ? 180 : 0)));

    this.inputSpeeds = defaultDrivingSpeeds;
    setDefaultCommand(driveCommand(inputSpeeds, true));

    // Initialize robot speed tracking and autonomous commands
    robotCurrentSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // autoDropdown = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Autonomous Selector", autoDropdown);

    this.setName("Swerve Drivetrain");
  }

  /**
   * Drives the robot with specified chassis speeds.
   *
   * @param speeds Desired chassis speeds.
   * @param fieldRelative Whether to interpret speeds relative to the field.
   * @param useOverrides Whether to apply driving overrides.
   */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean useOverrides) {
    this.robotCurrentSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                useOverrides ? (applyOverrides(speeds)) : (speeds), gyro.getRotation2d())
            : speeds;

    SwerveModuleState[] swerveModuleStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(this.robotCurrentSpeeds, 0.02).getChassisSpeeds());

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, config.getMaxSpeed());

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Creates a command for driving the robot using specified chassis speeds.
   *
   * @param speeds Supplier of chassis speeds.
   * @param fieldRelative Whether to interpret speeds relative to the field.
   * @return Command for driving the robot.
   */
  public Command driveCommand(Supplier<ChassisSpeeds> speeds, boolean fieldRelative) {
    Command out =
        new Command() {
          @Override
          public void execute() {
            drive(speeds.get(), fieldRelative, false);
          }
        };
    out.addRequirements(this);
    return out;
  }

  /**
   * Creates a command to drive the robot to a specified pose.
   *
   * @param xPoseMeters X-coordinate of the target pose in meters.
   * @param yPoseMeters Y-coordinate of the target pose in meters.
   * @param theta Rotation of the target pose.
   * @param constraints Path constraints for the autonomous path.
   * @return Command to drive the robot to the specified pose.
   */
  public Command driveToPose(
      Double xPoseMeters, Double yPoseMeters, Rotation2d theta, PathConstraints constraints) {
    return AutoBuilder.pathfindToPoseFlipped(
            new Pose2d(
                xPoseMeters == null ? odometry.getEstimatedPosition().getX() : xPoseMeters,
                yPoseMeters == null ? odometry.getEstimatedPosition().getY() : yPoseMeters,
                theta == null ? odometry.getEstimatedPosition().getRotation() : theta),
            constraints)
        .raceWith(
            Commands.runEnd(
                () -> {
                  autoOverrides =
                      new ChassisSpeeds(
                          xPoseMeters == null ? inputSpeeds.get().vxMetersPerSecond : null,
                          yPoseMeters == null ? inputSpeeds.get().vyMetersPerSecond : null,
                          theta == null ? inputSpeeds.get().omegaRadiansPerSecond : null);
                },
                () -> {
                  autoOverrides = new ChassisSpeeds();
                }));
  }

  /**
   * Applies driving overrides to the given chassis speeds.
   *
   * @param speeds Original chassis speeds.
   * @return Chassis speeds with overrides applied.
   */
  private ChassisSpeeds applyOverrides(ChassisSpeeds speeds) {
    return new ChassisSpeeds(
        autoOverrides.vxMetersPerSecond == 0
            ? speeds.vxMetersPerSecond
            : autoOverrides.vxMetersPerSecond,
        autoOverrides.vyMetersPerSecond == 0
            ? speeds.vyMetersPerSecond
            : autoOverrides.vyMetersPerSecond,
        autoOverrides.omegaRadiansPerSecond == 0
            ? speeds.omegaRadiansPerSecond
            : autoOverrides.omegaRadiansPerSecond);
  }

  @Override
  public void periodic() {
    odometry.update(
        this.gyro.getRotation2d(),
        new SwerveModulePosition[] {
          this.frontLeft.getModulePosition(),
          this.frontRight.getModulePosition(),
          this.backLeft.getModulePosition(),
          this.backRight.getModulePosition()
        });

    odometryField.setRobotPose(odometry.getEstimatedPosition());
    SmartDashboard.putData(gyro);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  /**
   * Gets the Field2d object for visualizing odometry.
   *
   * @return Field2d object for odometry visualization.
   */
  public Field2d getOdometryField() {
    return this.odometryField;
  }

  /**
   * Sets the robot's odometry position.
   *
   * @param position New odometry position.
   */
  public void setOdometryPosition(Pose2d position) {
    this.odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          this.frontLeft.getModulePosition(),
          this.frontRight.getModulePosition(),
          this.backLeft.getModulePosition(),
          this.backRight.getModulePosition()
        },
        position);
  }

  /**
   * Adds a vision measurement to the odometry.
   *
   * @param position Vision-based position measurement.
   * @param timestampSeconds Timestamp of the vision measurement.
   */
  public void addVisionMeasurement(Pose2d position, double timestampSeconds) {
    this.odometry.addVisionMeasurement(position, timestampSeconds);
  }

  /**
   * Gets the selected autonomous command from the dashboard.
   *
   * @return Selected autonomous command.
   */
  public Command getAutonomousCommand() {

    return autoDropdown.getSelected();
  }
}
