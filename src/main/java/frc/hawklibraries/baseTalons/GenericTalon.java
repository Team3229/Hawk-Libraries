package frc.hawklibraries.baseTalons;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class GenericTalon {
    private TalonFX motor;

    public GenericTalon(int canID, CANBus canBus, TalonConfig config) {

    }

    public void reapplyConfig(TalonFXConfiguration config) {
        motor.getConfigurator().apply(config);
    }
}
