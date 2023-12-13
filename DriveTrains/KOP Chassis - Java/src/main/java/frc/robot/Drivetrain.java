package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain {

    private static DifferentialDrive drivetrain;
    private static CANSparkMax left;
    private static CANSparkMax right;
    
    public static void init() {
        left = new CANSparkMax(1, MotorType.kBrushless);
        right = new CANSparkMax(2, MotorType.kBrushless);

        left.setInverted(false);
        right.setInverted(false);
        
        drivetrain = new DifferentialDrive(left, right);
    }

    public static void drive(double x, double y) {
        drivetrain.arcadeDrive(x, y);
    }

}