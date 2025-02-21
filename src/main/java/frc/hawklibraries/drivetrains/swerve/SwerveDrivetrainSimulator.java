package frc.hawklibraries.drivetrains.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrivetrainSimulator {
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator odometry;
    private Field2d field;
    private Timer timer;

    // Swerve module simulators
    private SwerveModuleSimulator frontLeft;
    private SwerveModuleSimulator frontRight;
    private SwerveModuleSimulator backLeft;
    private SwerveModuleSimulator backRight;

    public SwerveDrivetrainSimulator(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
        this.odometry =
        new SwerveDrivePoseEstimator(
            this.kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.7, 0.7, 9999999)
        );

        this.field = new Field2d();
        this.timer = new Timer();
        this.timer.start();
        SmartDashboard.putData("Field", field);

        // Initialize swerve module simulators
        this.frontLeft = new SwerveModuleSimulator();
        this.frontRight = new SwerveModuleSimulator();
        this.backLeft = new SwerveModuleSimulator();
        this.backRight = new SwerveModuleSimulator();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void update(double dt) {
        double currentTime = timer.get();

        // Update module simulators
        frontLeft.update(dt);
        frontRight.update(dt);
        backLeft.update(dt);
        backRight.update(dt);

        // Update module positions
        SwerveModulePosition[] modulePositions = {
            frontLeft.toSwerveModulePosition(),
            frontRight.toSwerveModulePosition(),
            backLeft.toSwerveModulePosition(),
            backRight.toSwerveModulePosition()
        };

        Pose2d currentPose = odometry.updateWithTime(currentTime, new Rotation2d(), modulePositions);
        field.setRobotPose(currentPose);
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
    }
}