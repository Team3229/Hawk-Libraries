package frc.robot.hawklibaries.drivetrains.tank;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hawklibaries.vendorRewrites.rev.CANSparkMax;

public class TankDrivetrain extends SubsystemBase {
    
    private TankDrivetrainConfig config;

    private DifferentialDriveKinematics m_kinematics;

    private CANSparkMax m_leftLead;
    private CANSparkMax m_leftFollow;
    private CANSparkMax m_RightLead;
    private CANSparkMax m_rightFollow;

    public TankDrivetrain(TankDrivetrainConfig config) {

        this.config = config;

        m_kinematics = new DifferentialDriveKinematics(this.config.getTrackWidth());

    }

}