package frc.robot.hawklibraries.drivetrains.swerve;

import frc.robot.hawklibraries.drivetrains.DrivetrainConfig;
import frc.robot.hawklibraries.utilities.PIDConstants;

public class SwerveDrivetrainConfig extends DrivetrainConfig {

    private double[] moduleOffsets;
    private double moduleDistance;
    private PIDConstants translationPID;
    private PIDConstants rotationPID;
    private SwerveModuleConfig frontLeftConfig;
    private SwerveModuleConfig frontRightConfig;
    private SwerveModuleConfig backLeftConfig;
    private SwerveModuleConfig backRightConfig;

    private SwerveDrivetrainConfig(Builder builder) {
        super(builder);
        this.moduleOffsets = builder.moduleOffsets;
        this.moduleDistance = builder.moduleDistance;
        this.translationPID = builder.translationPID;
        this.rotationPID = builder.rotationPID;
        this.frontLeftConfig = builder.frontLeftConfig;
        this.frontRightConfig = builder.frontRightConfig;
        this.backLeftConfig = builder.backLeftConfig;
        this.backRightConfig = builder.backRightConfig;
    }

    public double[] getModuleOffsets() {return this.moduleOffsets;}
    public double getModuleDistance() {return this.moduleDistance;}
    public PIDConstants getTranslationPID() {return this.translationPID;}
    public PIDConstants getRotationPID() {return this.rotationPID;}
    public SwerveModuleConfig getFrontLeftConfig() {return this.frontLeftConfig;}
    public SwerveModuleConfig getFrontRightConfig() {return this.frontRightConfig;}
    public SwerveModuleConfig getBackLeftConfig() {return this.backLeftConfig;}
    public SwerveModuleConfig getBackRightConfig() {return this.backRightConfig;}

    public static class Builder extends DrivetrainConfig.Builder<Builder> {

        private double[] moduleOffsets;
        private double moduleDistance;
        private PIDConstants translationPID;
        private PIDConstants rotationPID;
        private SwerveModuleConfig frontLeftConfig;
        private SwerveModuleConfig frontRightConfig;
        private SwerveModuleConfig backLeftConfig;
        private SwerveModuleConfig backRightConfig;

        public Builder moduleOffsets(double[] moduleOffsets) {
            this.moduleOffsets = moduleOffsets;
            return self();
        }

        public Builder moduleDistance(double moduleDistance) {
            this.moduleDistance = moduleDistance;
            return self();
        }

        public Builder translationPID(PIDConstants translationPID) {
            this.translationPID = translationPID;
            return self();
        }

        public Builder rotationPID(PIDConstants rotationPID) {
            this.rotationPID = rotationPID;
            return self();
        }

        public Builder frontLeftConfig(SwerveModuleConfig config) {
            this.frontLeftConfig = config;
            return self();
        }

        public Builder frontRightConfig(SwerveModuleConfig config) {
            this.frontRightConfig = config;
            return self();
        }

        public Builder backLeftConfig(SwerveModuleConfig config) {
            this.backLeftConfig = config;
            return self();
        }

        public Builder backRightConfig(SwerveModuleConfig config) {
            this.backRightConfig = config;
            return self();
        }

        @Override
        protected Builder self() {
            return this;
        }

        @Override
        public SwerveDrivetrainConfig build() {
            return new SwerveDrivetrainConfig(self());
        }
    }
}