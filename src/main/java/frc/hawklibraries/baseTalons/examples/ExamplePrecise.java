package frc.hawklibraries.baseTalons.examples;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.hawklibraries.baseTalons.PreciseTalon;
import frc.hawklibraries.baseTalons.TalonConfig;

public class ExamplePrecise extends SubsystemBase {
    // i am pretending that this is a subsystem, albiet a small one
    public PreciseTalon talon;
    public TalonConfig config;

    public ExamplePrecise() {
        super();
        config = new TalonConfig()
        .setPIDVASG(10, 1, 4, 45, 190, 0.0001, 85)
            .setSensorToMechanismRatio(1000)
            .setGravityModeValue(GravityTypeValue.Arm_Cosine)
            .setArmVelocity(25000)
            .setCurrentLimit(40) // the default is 40 so it won't be changed
            .setMaxVoltage(12) // the default is 12 so it won't be changed
            .setNeutralModeValue(NeutralModeValue.Coast)
            .setInvertedValue(InvertedValue.CounterClockwise_Positive)
            .applyConfigurators()
            .withPIDSendable("spin", talon.motor);

        talon = new PreciseTalon(0, CANBus.roboRIO(), config);
    }

    public Command moveTo(Angle angle) {
        moveTo(angle).addRequirements(this); // tbh idk if this is working, but i'd like it too
        return talon.rotateTo(angle);
    }

}
