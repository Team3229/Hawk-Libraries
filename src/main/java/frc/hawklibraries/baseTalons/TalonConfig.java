package frc.hawklibraries.baseTalons;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class TalonConfig {
    private TalonFXConfiguration talonConfig;
    private MotorOutputConfigs motorOutputConfigs;
    private CurrentLimitsConfigs currentLimitsConfigs;
    private FeedbackConfigs feedbackConfigs;
    private VoltageConfigs voltageConfigs;

    private MotionMagicConfigs motionMagicConfigs;
    private MotionMagicVoltage motionMagicVoltage;

    private Follower follower;

    private boolean inverted;

    private double p;
    private double i;
    private double d;
    private double v;
    private double a;
    private double s;
    private double g;
    private GravityTypeValue gravityTypeValue;

    public TalonConfig setP(double p) {
        this.p = p;
        return this;
    }
}
