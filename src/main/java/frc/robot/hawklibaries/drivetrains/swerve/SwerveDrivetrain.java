package frc.robot.hawklibaries.drivetrains.swerve;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hawklibaries.utilities.Alliance;
import frc.robot.hawklibaries.utilities.Alliance.AllianceColor;
import frc.robot.hawklibaries.vendorRewrites.wpilib.ChassisSpeeds;

public class SwerveDrivetrain extends SubsystemBase {
    
    private SwerveDrivetrainConfig config;

    private SwerveDriveKinematics m_kinematics;
    private SwerveDrivePoseEstimator m_odometry;
    private AHRS m_gyro;

    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;

    private Field2d m_odometryField;
    private ChassisSpeeds m_robotCurrentSpeeds;

    private SendableChooser<Command> autoDropdown;

    private ChassisSpeeds autoOverrides;
    private Supplier<ChassisSpeeds> inputSpeeds;

    public SwerveDrivetrain(
        SwerveDrivetrainConfig config,
        Supplier<ChassisSpeeds> defaultDrivingSpeeds
    ) {

        this.config = config;

        this.m_frontLeft = new SwerveModule(this.config.getFrontLeftConfig());
        this.m_frontRight = new SwerveModule(this.config.getFrontRightConfig());
        this.m_backLeft = new SwerveModule(this.config.getBackLeftConfig());
        this.m_backRight = new SwerveModule(this.config.getBackRightConfig());

        this.m_kinematics = new SwerveDriveKinematics(
            new Translation2d(
                this.config.getModuleDistance(),
                this.config.getModuleDistance()
            ),
            new Translation2d(
                this.config.getModuleDistance(),
                -this.config.getModuleDistance()
            ),
            new Translation2d(
                -this.config.getModuleDistance(),
                this.config.getModuleDistance()
            ),
            new Translation2d(
                -this.config.getModuleDistance(),
                -this.config.getModuleDistance()
            )
        );

        this.m_odometry = new SwerveDrivePoseEstimator(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                this.m_frontLeft.getModulePosition(),
                this.m_frontRight.getModulePosition(),
                this.m_backLeft.getModulePosition(),
                this.m_backRight.getModulePosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.7, 0.7, 9999999)
        );

        this.m_gyro = new AHRS();
        
        Alliance.alliancePresentTrigger().onTrue(
            Commands.runOnce(
                () -> {
                        m_gyro.setAngleAdjustment(
                            (Alliance.getAlliance() == AllianceColor.Red)
                                ? 180
                                : 0
                        );
                    }
                )  
        );

        this.inputSpeeds = defaultDrivingSpeeds;

        setDefaultCommand(driveCommand(inputSpeeds, true));

        m_robotCurrentSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        AutoBuilder.configureHolonomic(
            m_odometry::getEstimatedPosition,
            this::setOdometryPosition,
            () -> {
                return this.m_robotCurrentSpeeds.getChassisSpeeds();
            },
            (edu.wpi.first.math.kinematics.ChassisSpeeds speeds) -> {
                this.drive(new ChassisSpeeds(speeds), false, true);
            },
            new HolonomicPathFollowerConfig(
                new PIDConstants(
                    this.config.getTranslationPID().getkP(),
                    this.config.getTranslationPID().getkI(),
                    this.config.getTranslationPID().getkD()
                ),
                new PIDConstants(
                    this.config.getRotationPID().getkP(),
                    this.config.getRotationPID().getkI(),
                    this.config.getRotationPID().getkD()
                ),
                this.config.getMaxSpeed(),
                this.config.getModuleDistance() * 2,
                new ReplanningConfig()
            ),
            () -> Alliance.getAlliance() == AllianceColor.Red,
            this
        );

        autoDropdown = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous Selector", autoDropdown);

        this.setName("Swerve Drivetrain");

    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean useOverrides) {

        this.m_robotCurrentSpeeds = 
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    useOverrides
                        ? (applyOverrides(speeds))
                        : (speeds)
                    ,
                    m_gyro.getRotation2d()
                )
                : speeds;

        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                this.m_robotCurrentSpeeds,
                0.02
            ).getChassisSpeeds()
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, config.getMaxSpeed());

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public Command driveCommand(Supplier<ChassisSpeeds> speeds, boolean fieldRelative) {
        Command out = new Command() {
            @Override public void execute() {
                drive(speeds.get(), fieldRelative, false);
            }
        };
        out.addRequirements(this);
        return out;
    }

    public Command driveToPose(Double xPoseMeters, Double yPoseMeters, Rotation2d theta, PathConstraints constraints) {
        return AutoBuilder.pathfindToPoseFlipped(
            new Pose2d(
                xPoseMeters == null
                    ? m_odometry.getEstimatedPosition().getX()
                    : xPoseMeters,
                yPoseMeters == null
                    ? m_odometry.getEstimatedPosition().getY()
                    : yPoseMeters,
                theta == null
                    ? m_odometry.getEstimatedPosition().getRotation()
                    : theta
            ),
            constraints
        ).raceWith(
            Commands.runEnd(
                () -> {
                    autoOverrides = new ChassisSpeeds(
                        xPoseMeters == null
                            ? inputSpeeds.get().vxMetersPerSecond
                            : null,
                        yPoseMeters == null
                            ? inputSpeeds.get().vyMetersPerSecond
                            : null,
                        theta == null
                            ? inputSpeeds.get().omegaRadiansPerSecond
                            : null
                    );
                },
                () -> {
                    autoOverrides = new ChassisSpeeds();
                }
            )
        );
    }
    
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
                : autoOverrides.omegaRadiansPerSecond
        );
    }

    @Override
    public void periodic() {
        m_odometry.update(
            this.m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                this.m_frontLeft.getModulePosition(),
                this.m_frontRight.getModulePosition(),
                this.m_backLeft.getModulePosition(),
                this.m_backRight.getModulePosition()
            }
        );

        m_odometryField.setRobotPose(m_odometry.getEstimatedPosition());
        SmartDashboard.putData(m_gyro);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

    }

    public Field2d getOdometryField() {
        return this.m_odometryField;
    }

    public void setOdometryPosition(Pose2d position) {
        this.m_odometry.resetPosition(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                this.m_frontLeft.getModulePosition(),
                this.m_frontRight.getModulePosition(),
                this.m_backLeft.getModulePosition(),
                this.m_backRight.getModulePosition()
            },
            position
        );
    }

    public void addVisionMeasurement(Pose2d position, double timestampSeconds) {
        this.m_odometry.addVisionMeasurement(position, timestampSeconds);
    }

    public Command getAutonomousCommand() {
        return autoDropdown.getSelected();
    }

}