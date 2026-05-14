package frc.hawklibraries.baseTalons;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GenericTalon {
    public TalonFX motor;

    public GenericTalon(int canID, CANBus canBus, TalonConfig config) {
        motor = new TalonFX(canID, canBus);
        config.applyConfig(motor);
    }

    public void reapplyConfig(TalonConfig config) {
        config.applyConfig(motor);
    }
}
