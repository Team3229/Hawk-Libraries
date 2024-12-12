package frc.hawklibraries.vendorRewrites.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.hawklibraries.utilities.PIDConstants;
import frc.hawklibraries.vendorRewrites.wpilib.PIDController;

public class CANSparkMax extends SubsystemBase {

    public static enum SetpointType {
        kPosition,
        kVelocity,
        kDutyCycle
    }

    private boolean isSim;
    
    private com.revrobotics.CANSparkMax m_sparkMax;
    private RelativeEncoder m_encoder;
    private SparkPIDController m_pidController;

    private PIDConstants pidSim;
    private PIDController pidControllerSim;
    private SetpointType setpointTypeSim;
    private double positionSim;
    private double velocitySim;
    private double positionConversionFactorSim = 1;
    private double velocityConversionFactorSim = 1;
    private boolean invertedSim;
    
    private double updateRateSim = 0.02;
    private double freeSpeedSim = 5600;

    private Double currentSetpoint;

    public CANSparkMax(int deviceId, MotorType motorType) {
        super();
        isSim = RobotBase.isSimulation();
            
        if (!isSim) {
            this.m_sparkMax = new com.revrobotics.CANSparkMax(deviceId, motorType);
            this.m_encoder = m_sparkMax.getEncoder();
            this.m_pidController = m_sparkMax.getPIDController();
        } else {

        }
    }

    public void setPID(PIDConstants pid) {
        if (!isSim) {
            this.m_pidController.setP(pid.getkP());
            this.m_pidController.setI(pid.getkI());
            this.m_pidController.setD(pid.getkD());
            this.m_pidController.setFF(pid.getkFF());
        } else {
            setPIDSim(pid);
        }
    }

    private void setPIDSim(PIDConstants pid) {
        this.pidSim = pid;
        this.pidControllerSim.setPID(pidSim.getkP(), pidSim.getkI(), pidSim.getkD());
    }

    public void setSetpoint(double setpoint, SetpointType type) {
        if (!isSim) {
            if (type == SetpointType.kPosition) {
                this.currentSetpoint = setpoint;
                this.m_pidController.setReference(setpoint, ControlType.kPosition);
            }

            if (type == SetpointType.kVelocity) {
                this.currentSetpoint = setpoint;
                this.m_pidController.setReference(setpoint, ControlType.kVelocity);
            }
        } else {
            setSetpointSim(setpoint, type);
        }
    }

    private void setSetpointSim(double setpoint, SetpointType type) {
        if (type == SetpointType.kPosition) {
            this.currentSetpoint = setpoint;
            this.setpointTypeSim = type;
            this.pidControllerSim.setSetpoint(setpoint);
        }

        if (type == SetpointType.kVelocity) {
            this.currentSetpoint = setpoint;
            this.setpointTypeSim = type;
            this.pidControllerSim.setSetpoint(setpoint);
        }
    }

    public void enablePositionWrapping(double min, double max) {
        if (!isSim) {
            this.m_pidController.setPositionPIDWrappingMinInput(min);
            this.m_pidController.setPositionPIDWrappingMaxInput(max);
            this.m_pidController.setPositionPIDWrappingEnabled(true);
        } else {
            enablePositionWrappingSim(min, max);
        }
    }

    private void enablePositionWrappingSim(double min, double max) {
        this.pidControllerSim.enableContinuousInput(min, max);
    }

    public void disablePositionWrapping() {
        if (!isSim) {
            this.m_pidController.setPositionPIDWrappingEnabled(false);
        } else {
            disablePositionWrappingSim();
        }
    }

    private void disablePositionWrappingSim() {
        this.pidControllerSim.disableContinuousInput();
    }

    public void set(double speed) {
        if (!isSim) {
            this.currentSetpoint = speed;
            this.m_pidController.setReference(speed, ControlType.kDutyCycle);
        } else {
            setSim(speed);
        }
    }

    private void setSim(double speed) {
        this.currentSetpoint = speed;
        this.setpointTypeSim = SetpointType.kDutyCycle;
        this.pidControllerSim.setSetpoint(speed);
    }

    public void stopMotor() {
        if (!isSim) {
            this.currentSetpoint = null;
            this.m_pidController.setReference(0, ControlType.kDutyCycle);
        } else {
            stopMotorSim();
        }
    }

    private void stopMotorSim() {
        this.currentSetpoint = null;
        this.setpointTypeSim = SetpointType.kDutyCycle;
        this.pidControllerSim.setSetpoint(0);
    }

    public void restoreFactoryDefaults() {
        if (!isSim) {
            this.m_sparkMax.restoreFactoryDefaults();
        }
    }

    public Double getSetpoint() {
        return this.currentSetpoint;
    }

    public double getPosition() {
        if (!isSim) {
            return this.m_encoder.getPosition();
        } else {
            return getPositionSim();
        }
    }

    private double getPositionSim() {
        return positionSim;
    }

    public double getVelocity() {
        if (!isSim) {
            return this.m_encoder.getVelocity();
        } else {
            return getVelocitySim();
        }
    }

    private double getVelocitySim() {
        return velocitySim;
    }

    public void setPositionConversionFactor(double conversionFactor) {
        if (!isSim) {
            this.m_encoder.setPositionConversionFactor(conversionFactor);
        } else {
            setPositionConversionFactorSim(conversionFactor);
        }
    }

    private void setPositionConversionFactorSim(double conversionFactor) {
        positionConversionFactorSim = conversionFactor;
    }

    public void setVelocityConversionFactor(double conversionFactor) {
        if (!isSim) {
            this.m_encoder.setVelocityConversionFactor(conversionFactor);
        } else {
            setVelocityConversionFactorSim(conversionFactor);
        }
    }

    private void setVelocityConversionFactorSim(double conversionFactor) {
        velocityConversionFactorSim = conversionFactor;
    }

    public void setPosition(double position) {
        if (!isSim) {
            this.m_encoder.setPosition(position);
        } else {
            setPositionSim(position);
        }
    }

    private void setPositionSim(double position) {
        positionSim = position;
    }

    public void setIdleMode(IdleMode idleMode) {
        if (!isSim) {
            this.m_sparkMax.setIdleMode(idleMode);
        } else {
            setIdleModeSim(idleMode);
        }
    }

    private void setIdleModeSim(IdleMode idleMode) {

    }

    public void setInverted(boolean inverted) {
        if (!isSim) {
            this.m_sparkMax.setInverted(inverted);
        } else {
            setInvertedSim(inverted);
        }
    }

    private void setInvertedSim(boolean inverted) {
        invertedSim = inverted;
    }

    public void setOutputRange(double min, double max) {
        if (!isSim) {
            this.m_pidController.setOutputRange(min, max);
        } else {
            setOutputRangeSim(min, max);
        }
    }

    private void setOutputRangeSim(double min, double max) {
        pidControllerSim.setOutputRange(min, max);
    }

    @Override
    public void periodic() {
        if (isSim) {
            // Update position and velocity to simulate motor movement
            switch (setpointTypeSim) {
                case kDutyCycle:
                    velocitySim = currentSetpoint * freeSpeedSim * velocityConversionFactorSim * ((invertedSim) ? -1 : 1);
                    positionSim += velocitySim * updateRateSim * positionConversionFactorSim;
                    break;
                
                case kPosition:
                    velocitySim = pidControllerSim.calculate(positionSim) * freeSpeedSim * velocityConversionFactorSim * ((invertedSim) ? -1 : 1);
                    positionSim += velocitySim * updateRateSim * positionConversionFactorSim;
                    break;

                case kVelocity:
                    velocitySim = pidControllerSim.calculate(velocitySim) * freeSpeedSim * velocityConversionFactorSim * ((invertedSim) ? -1 : 1);
                    positionSim += velocitySim * updateRateSim * positionConversionFactorSim;
                    break;
            
                default:
                    break;
            }

        }
    }

}
