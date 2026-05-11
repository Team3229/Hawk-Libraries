package frc.hawklibraries.baseTalons.examples;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.hawklibraries.baseTalons.PerpetualTalon;
import frc.hawklibraries.baseTalons.TalonConfig;

public class ExamplePerpetual extends SubsystemBase {
    // i am pretending that this is a subsystem, albiet a small one
    public PerpetualTalon talon;
    public TalonConfig config;

    public ExamplePerpetual() {
        super();
        config = new TalonConfig()
            .setPVS(1, 1, 1)
            .setSensorToMechanismRatio(1.5)
            .setCurrentLimit(40) // the default is 40 so it won't be changed
            .setMaxVoltage(12) // the default is 12 so it won't be changed
            .setNeutralModeValue(NeutralModeValue.Coast)
            .setInvertedValue(InvertedValue.CounterClockwise_Positive)
            .applyConfigurators()
            .withPIDSendable("spin", talon.motor);

        talon = new PerpetualTalon(0, CANBus.roboRIO(), config);
    }

    public Command spin(double speed) {
        spin(speed).addRequirements(this);
        return talon.spinMotor(speed);
    }
}
