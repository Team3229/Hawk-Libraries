package frc.hawklibraries.baseTalons;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Velocity;


public class TalonConfig {
    private TalonFXConfiguration talonConfig;
    private MotorOutputConfigs motorOutputConfigs;
    private CurrentLimitsConfigs currentLimitsConfigs;
    private FeedbackConfigs feedbackConfigs;
    private VoltageConfigs voltageConfigs;
    private Slot0Configs slot0Configs;

    private MotionMagicConfigs motionMagicConfigs;

    private InvertedValue forwardDirection;
    private NeutralModeValue neutralModeValue;
    private double maxVoltage = 12;
    private double sensorToMechanismRatio;

    private double armVelocity;

    private double p;
    private double i;
    private double d;
    private double v;
    private double a;
    private double s;
    private double g;
    private GravityTypeValue gravityTypeValue;

    private double currentLimit = 40;

    public TalonConfig setPIDVASG(double p, double i, double d, double v, double a, double s, double g) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.v = v;
        this.a = a;
        this.s = s;
        this.g = g;

        return this;
    }

    public TalonConfig setPID(double p, double i, double d) {
        return setPIDVASG(p, i, d, this.v, this.a, this.s, this.g);
    }
    
    public TalonConfig setVASG(double v, double a, double s, double g) {
        return setPIDVASG(this.p, this.i, this.d, v, a, s, g);
    }

    public TalonConfig setPVS(double p, double v, double s) {
        return setPIDVASG(p, this.i, this.d, v, this.a, s, this.g);   
    }

    public TalonConfig setInvertedValue(InvertedValue forwardDirection) {
        this.forwardDirection = forwardDirection;
        return this;
    }
    
    public TalonConfig setMaxVoltage(double maxVoltage) {
        this.maxVoltage = maxVoltage;
        return this;
    }

    public TalonConfig setNeutralModeValue(NeutralModeValue neutralModeValue) {
        this.neutralModeValue = neutralModeValue;
        return this;
    }

    public TalonConfig setGravityModeValue(GravityTypeValue gravityTypeValue) {
        this.gravityTypeValue = gravityTypeValue;
        return this;
    }

    public TalonConfig setSensorToMechanismRatio(double sensorToMechanismRatio) {
        this.sensorToMechanismRatio = sensorToMechanismRatio;
        return this;
    }

    public TalonConfig setCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public TalonConfig setArmVelocity(double armVelocity) {
        this.armVelocity = armVelocity;
        return this;
    }
    
    public TalonConfig talonConfig() {
        talonConfig = new TalonFXConfiguration();

        return this;
    }

    public TalonConfig setSlot0() {
        slot0Configs = new Slot0Configs()
            .withGravityType(gravityTypeValue)
            .withKP(p)
            .withKI(i)
            .withKD(d)
            .withKV(v)
            .withKA(a)
            .withKS(s)
            .withKG(g);
        talonConfig.withSlot0(slot0Configs);
        
        return this;
    }

    public TalonConfig withFeedbackConfigurator() {
        feedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(sensorToMechanismRatio);
        talonConfig.withFeedback(feedbackConfigs);

        return this;
    }

    public TalonConfig withVoltageConfigurator() {
        voltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(maxVoltage)
            .withPeakReverseVoltage(-maxVoltage);
        talonConfig.withVoltage(voltageConfigs);
        
        return this;
    }

    public TalonConfig withMotorOutputConfigurator() {
        motorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(neutralModeValue)
            .withInverted(forwardDirection);
        talonConfig.withMotorOutput(motorOutputConfigs);
        
        return this;
    }

    public TalonConfig withCurrentLimitsConfigurator() {
        currentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(currentLimit)
            .withSupplyCurrentLimitEnable(true);
        talonConfig.withCurrentLimits(currentLimitsConfigs);
        
        return this;
    }

    public TalonConfig withMotionMagicConfigurator(){
        motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(armVelocity)
            .withMotionMagicAcceleration(armVelocity * 2)
            .withMotionMagicJerk(armVelocity * 20);
        talonConfig.withMotionMagic(motionMagicConfigs);

        return this;
    }

    public void applyConfig(TalonFX motor) {
        motor.getConfigurator().apply(talonConfig);
    }
    
}
