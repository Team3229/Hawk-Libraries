package frc.hawklibraries.baseTalons;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * motor that intends to reach and maintain an angle
 */
public class PreciseTalon extends GenericTalon {

    public PreciseTalon(int canID, CANBus canBus, TalonConfig talonConfig) {
        super(canID, canBus, talonConfig);
    }

    public Command rotateTo(Angle desiredAngle) {
        Command out = new Command() {
            @Override 
            public void execute() {
                motor.setControl(new MotionMagicVoltage(desiredAngle).withSlot(0));
            }
            @Override
            public void end(boolean interrupted) {
                motor.setControl(new StaticBrake());
            }
        };
        return out; 
    }

}