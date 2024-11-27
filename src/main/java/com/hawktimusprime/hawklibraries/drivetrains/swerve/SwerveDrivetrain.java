package com.hawktimusprime.hawklibraries.drivetrains.swerve;

import java.util.NoSuchElementException;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {
    
    private SwerveDrivetrainConfig config;

    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator odometry;
    private AHRS gyro;

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    private Field2d odometryField;
    private ChassisSpeeds robotCurrentSpeeds;

    private SendableChooser<Command> autoDropdown;

    public SwerveDrivetrain(
        SwerveDrivetrainConfig config,
        Supplier<ChassisSpeeds> defaultDrivingSpeeds
    ) {

        this.config = config;

        this.frontLeft = new SwerveModule(this.config.getFrontLeftConfig());
        this.frontRight = new SwerveModule(this.config.getFrontRightConfig());
        this.backLeft = new SwerveModule(this.config.getBackLeftConfig());
        this.backRight = new SwerveModule(this.config.getBackRightConfig());

        this.kinematics = new SwerveDriveKinematics(
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

        this.odometry = new SwerveDrivePoseEstimator(
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
            VecBuilder.fill(0.7, 0.7, 9999999)
        );

        this.gyro = new AHRS();
        new Command() {
            @Override public void execute() {
                DriverStation.getAlliance().ifPresent((Alliance a) -> {
                    gyro.setAngleAdjustment(
                        (a == Alliance.Red)
                            ? 180
                            : 0
                        );
                    }
                );
            }

            @Override public boolean isFinished() {return DriverStation.getAlliance().isPresent();}
        }.schedule();

        setDefaultCommand(driveCommand(defaultDrivingSpeeds, true));

        robotCurrentSpeeds = new ChassisSpeeds();

        AutoBuilder.configureHolonomic(
            odometry::getEstimatedPosition,
            this::setOdometryPosition,
            () -> {
                return this.robotCurrentSpeeds;
            },
            (ChassisSpeeds speeds) -> {
                this.drive(speeds, false);
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
            () -> this.getAlliance() == Alliance.Red,
            this
        );

        autoDropdown = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous Selector", autoDropdown);

    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {

        this.robotCurrentSpeeds = 
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    gyro.getRotation2d()
                )
                : speeds;

        var swerveModuleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                this.robotCurrentSpeeds,
                0.02
            )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, config.getMaxSpeed());

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    public Command driveCommand(Supplier<ChassisSpeeds> speeds, boolean fieldRelative) {
        Command out = new Command() {
            @Override public void execute() {
                drive(speeds.get(), fieldRelative);
            }
        };
        out.addRequirements(this);
        return out;
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
            }
        );

        odometryField.setRobotPose(odometry.getEstimatedPosition());
        SmartDashboard.putData(gyro);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

    }

    public Field2d getOdometryField() {
        return this.odometryField;
    }

    public void setOdometryPosition(Pose2d position) {
        this.odometry.resetPosition(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                this.frontLeft.getModulePosition(),
                this.frontRight.getModulePosition(),
                this.backLeft.getModulePosition(),
                this.backRight.getModulePosition()
            },
            position
        );
    }

    public Command getAutonomousCommand() {
        return autoDropdown.getSelected();
    }

    private Alliance getAlliance() {

        try {
            return DriverStation.getAlliance().get();   
        } catch (NoSuchElementException e) {
            return Alliance.Blue;
        }

    }

}