package frc.robot.hawklibaries.drivetrains.tank;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.hawklibaries.drivetrains.DrivetrainConfig;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.SuperBuilder;

@SuperBuilder
@Getter
@Setter
public class TankDrivetrainConfig extends DrivetrainConfig {

    private int leftLeadID;
    private int leftFollowerID;
    private int rightLeadID;
    private int rightFollowerID;
    @Builder.Default
    private MotorType motorType = MotorType.kBrushless;
    private double trackWidth;

}