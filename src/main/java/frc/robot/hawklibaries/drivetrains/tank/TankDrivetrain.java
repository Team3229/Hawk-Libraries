package frc.robot.hawklibaries.drivetrains.tank;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrivetrain extends SubsystemBase {
    
    private TankDrivetrainConfig config;

    private DifferentialDriveKinematics kinematics;
    private DifferentialDrivePoseEstimator odometry;

    private CANSparkMax leftLeadMotor;
    private CANSparkMax leftFollowerMotor;
    private CANSparkMax rightLeadMotor;
    private CANSparkMax rightFollowerMotor;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private AHRS navX;

    private Field2d odometryField;

    private ChassisSpeeds robotCurrentSpeeds;

    public TankDrivetrain(
        TankDrivetrainConfig config,
        Supplier<ChassisSpeeds> defaultDrivingSpeeds
        ) {
        
        this.config = config;

        leftLeadMotor = new CANSparkMax(this.config.getLeftLeadID(), this.config.getMotorType());
        leftFollowerMotor = new CANSparkMax(this.config.getLeftFollowerID(), this.config.getMotorType());
        rightLeadMotor = new CANSparkMax(this.config.getRightLeadID(), this.config.getMotorType());
        rightFollowerMotor = new CANSparkMax(this.config.getRightFollowerID(), this.config.getMotorType());

        leftFollowerMotor.follow(leftLeadMotor);
        rightFollowerMotor.follow(rightLeadMotor);

        leftEncoder = leftLeadMotor.getEncoder();
        rightEncoder = rightLeadMotor.getEncoder();

        navX = new AHRS();

        kinematics = new DifferentialDriveKinematics(config.getTrackWidth());
        odometry = new DifferentialDrivePoseEstimator(
            kinematics,
            navX.getRotation2d(),
            leftEncoder.getPosition(),
            rightEncoder.getPosition(),
            new Pose2d()
        );

        odometryField = new Field2d();
        robotCurrentSpeeds = new ChassisSpeeds();

        setDefaultCommand(driveCommand(defaultDrivingSpeeds));

    }

    private Command driveCommand(Supplier<ChassisSpeeds> speeds) {
        Command out = new Command() {
            @Override public void execute() {
                drive(speeds.get());
            }
        };

        out.addRequirements(this);
        return out;
    }

    public void drive(ChassisSpeeds speeds) {
        robotCurrentSpeeds = speeds;

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(robotCurrentSpeeds);
        wheelSpeeds.desaturate(config.getMaxSpeed());

        leftLeadMotor.set(wheelSpeeds.leftMetersPerSecond);
        rightLeadMotor.set(wheelSpeeds.rightMetersPerSecond);

    }

}
