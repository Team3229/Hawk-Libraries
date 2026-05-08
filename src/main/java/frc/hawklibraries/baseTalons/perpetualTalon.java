package frc.hawklibraries.baseTalons;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * motor that intends to constantly spin during an interval
 */
public class PerpetualTalon extends GenericTalon{

    public PerpetualTalon(int canID, CANBus canBus, TalonConfig talonConfig) {
        super(canID, canBus, talonConfig);
    }

    public Command spinMotor(double desiredSpeed) {
        Command out = new Command() {

            @Override
            public void execute() {
                motor.setControl(new VelocityVoltage(desiredSpeed).withSlot(0));
            }

            @Override
            public void end(boolean interrupted) {
                motor.setControl(new CoastOut());
            }

        };

        return out;

    }
}
