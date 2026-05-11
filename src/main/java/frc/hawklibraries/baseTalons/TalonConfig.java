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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


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

    private Sendable pidSetter;

    /**
     * Sets the P, I, D, V, A, S, G values without setting the values on a motor.
     */
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

    /**
     * Sets the P, I, D, V, A, S, G values with setting the values on a motor.
     */
    public TalonConfig setPIDVASG(double p, double i, double d, double v, double a, double s, double g, TalonFX talon) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.v = v;
        this.a = a;
        this.s = s;
        this.g = g;
        setSlot0();
        applyConfig(talon);
        
        return this;
    }

    /**
     * Sets the P, I, D values with setting the values on a motor.
     */
    public TalonConfig setPID(double p, double i, double d, TalonFX talon) {
        return setPIDVASG(p, i, d, this.v, this.a, this.s, this.g, talon);
    }
    
    /**
     * Sets the V, A, S, G values with setting the values on a motor.
     */
    public TalonConfig setVASG(double v, double a, double s, double g, TalonFX talon) {
        return setPIDVASG(this.p, this.i, this.d, v, a, s, g, talon);
    }

    /**
     * Sets the P, V, S values with setting the values on a motor.
     */
    public TalonConfig setPVS(double p, double v, double s, TalonFX talon) {
        return setPIDVASG(p, this.i, this.d, v, this.a, s, this.g, talon);
    }

    /**
     * Sets the P, I, D values without setting the values on a motor.
     */
    public TalonConfig setPID(double p, double i, double d) {
        return setPIDVASG(p, i, d, this.v, this.a, this.s, this.g);
    }
    
    /**
     * Sets the V, A, S, G values without setting the values on a motor.
     */
    public TalonConfig setVASG(double v, double a, double s, double g) {
        return setPIDVASG(this.p, this.i, this.d, v, a, s, g);
    }

    /**
     * Sets the P, V, S values with setting the values on a motor.
     */
    public TalonConfig setPVS(double p, double v, double s) {
        return setPIDVASG(p, this.i, this.d, v, this.a, s, this.g);
    }

    /**
     * Sets the P value to the motor
     */
    public TalonConfig setP(double p, TalonFX talon) {
        return setPID(p, this.i, this.d, talon);
    }

    /**
     * Sets the I value to the motor
     */
    public TalonConfig setI(double i, TalonFX talon) {
        return setPID(this.p, i, this.d, talon);   
    }

    /**
     * Sets the D value to the motor
     */
    public TalonConfig setD(double d, TalonFX talon) {
        return setPID(this.p, this.i, d, talon);   
    }

    /**
     * Sets the V value to the motor
     */
    public TalonConfig setV(double v, TalonFX talon) {
        return setVASG(v, this.a, this.s, this.g, talon);   
    }

    /**
     * Sets the A value to the motor
     */
    public TalonConfig setA(double a, TalonFX talon) {
        return setVASG(this.v, a, this.s, this.g, talon);   
    }

    /**
     * Sets the S value to the motor
     */
    public TalonConfig setS(double s, TalonFX talon) {
        return setVASG(this.v, this.a, s, this.g, talon);   
    }

    /**
     * Sets the G value to the motor
     */
    public TalonConfig setG(double g, TalonFX talon) {
        return setVASG(this.v, this.a, this.s, g, talon);   
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

    public TalonConfig withMotionMagicConfigurator() {
        motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(armVelocity)
            .withMotionMagicAcceleration(armVelocity * 2)
            .withMotionMagicJerk(armVelocity * 20);
        talonConfig.withMotionMagic(motionMagicConfigs);

        return this;
    }

    public TalonConfig applyConfigurators() {
        withMotionMagicConfigurator();
        withCurrentLimitsConfigurator();
        withFeedbackConfigurator();
        withMotorOutputConfigurator();
        withVoltageConfigurator();
        setSlot0();

        return this;
    }

    public TalonConfig withPIDSendable(String name, TalonFX talon) {
        pidSetter = new Sendable() {
		    @Override
			public void initSendable(SendableBuilder builder) {
				builder.addDoubleProperty("P", () -> p, (P) -> setP(P, talon));
				builder.addDoubleProperty("I", () -> i, (I) -> setI(I, talon));
				builder.addDoubleProperty("D", () -> d, (D) -> setD(D, talon));
				builder.addDoubleProperty("V", () -> v, (V) -> setV(V, talon));
				builder.addDoubleProperty("A", () -> a, (A) -> setA(A, talon));
				builder.addDoubleProperty("S", () -> s, (S) -> setS(S, talon));
				builder.addDoubleProperty("G", () -> g, (G) -> setG(G, talon));
			}
		};
		SmartDashboard.putData(name + "PID", pidSetter);
        return this;
    }


    public void applyConfig(TalonFX motor) {
        motor.getConfigurator().apply(talonConfig);
    }
    
}
