package frc.robot.hawklibaries.drivetrains.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hawklibaries.utilities.PIDConstants;

public class SwerveModuleConfig {
    
    private double wheelDiameter;
    private double driveGearRatio;
    private double turningGearRatio;
    private double maxSpeed;
    private int driveID;
    private int turningID;
    private int turningEncoderID;
    private Rotation2d encoderOffset;
    private PIDConstants drivePID;
    private PIDConstants turningPID;
    private double turningOutputMin;
    private double turningOutputMax;

    private SwerveModuleConfig(Builder builder) {
        
        this.wheelDiameter = builder.wheelDiameter;
        this.driveGearRatio = builder.driveGearRatio;
        this.turningGearRatio = builder.turningGearRatio;
        this.maxSpeed = builder.maxSpeed;
        this.driveID = builder.driveID;
        this.turningID = builder.turningID;
        this.turningEncoderID = builder.turningEncoderID;
        this.encoderOffset = builder.encoderOffset;
        this.drivePID = builder.drivePID;
        this.turningPID = builder.turningPID;
        this.turningOutputMin = builder.turningOutputMin;
        this.turningOutputMax = builder.turningOutputMax;

    }

    public double getWheelDiameter() {return this.wheelDiameter;}
    public double getDriveGearRatio() {return this.driveGearRatio;}
    public double getTurningGearRatio() {return this.turningGearRatio;}
    public double getMaxSpeed() {return this.maxSpeed;}
    public int getDriveID() {return this.driveID;}
    public int getTurningID() {return this.turningID;}
    public int getTurningEncoderID() {return this.turningEncoderID;}
    public Rotation2d getEncoderOffset() {return this.encoderOffset;}
    public PIDConstants getDrivePID() {return this.drivePID;}
    public PIDConstants getTurningPID() {return this.turningPID;}
    public double getTurningOutputMin() {return this.turningOutputMin;}
    public double getTurningOutputMax() {return this.turningOutputMax;}

    public static class Builder {
        
        private double wheelDiameter = 0.25;
        private double driveGearRatio = 1.0;
        private double turningGearRatio = 1.0;
        private double maxSpeed = 3.0;
        private int driveID;
        private int turningID;
        private int turningEncoderID;
        private Rotation2d encoderOffset;
        private PIDConstants drivePID;
        private PIDConstants turningPID;
        private double turningOutputMin;
        private double turningOutputMax;

        public Builder wheelDiameter(double wheelDiameter) {
            this.wheelDiameter = wheelDiameter;
            return this;
        }

        public Builder driveGearRatio(double driveGearRatio) {
            this.driveGearRatio = driveGearRatio;
            return this;
        }

        public Builder turningGearRatio(double turningGearRatio) {
            this.turningGearRatio = turningGearRatio;
            return this;
        }

        public Builder maxSpeed(double maxSpeed) {
            this.maxSpeed = maxSpeed;
            return this;
        }

        public Builder driveID(int driveID) {
            this.driveID = driveID;
            return this;
        }

        public Builder turningID(int turningID) {
            this.turningID = turningID;
            return this;
        }

        public Builder turningEncoderID(int turningEncoderID) {
            this.turningEncoderID = turningEncoderID;
            return this;
        }

        public Builder encoderOffset(Rotation2d encoderOffset) {
            this.encoderOffset = encoderOffset;
            return this;
        }

        public Builder drivePID(PIDConstants drivePID) {
            this.drivePID = drivePID;
            return this;
        }

        public Builder turningPID(PIDConstants turningPID) {
            this.turningPID = turningPID;
            return this;
        }

        public Builder turningOutputMin(double turningOutputMin) {
            this.turningOutputMin = turningOutputMin;
            return this;
        }

        public Builder turningOutputMax(double turningOutputMax) {
            this.turningOutputMax = turningOutputMax;
            return this;
        }

        public SwerveModuleConfig build() {
            return new SwerveModuleConfig(this);
        }
    }
}