package frc.robot.hawklibaries.drivetrains.swerve;

import frc.robot.hawklibaries.drivetrains.DrivetrainConfig;
import frc.robot.hawklibaries.utilities.PIDConstants;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.SuperBuilder;

@SuperBuilder
@Getter
@Setter
public class SwerveDrivetrainConfig extends DrivetrainConfig {

    private double[] moduleOffsets;
    private double moduleDistance;
    private PIDConstants translationPID;
    private PIDConstants rotationPID;
    private SwerveModuleConfig frontLeftConfig;
    private SwerveModuleConfig frontRightConfig;
    private SwerveModuleConfig backLeftConfig;
    private SwerveModuleConfig backRightConfig;

}