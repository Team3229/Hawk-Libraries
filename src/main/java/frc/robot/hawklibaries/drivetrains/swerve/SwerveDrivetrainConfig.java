package frc.robot.hawklibaries.drivetrains.swerve;

import frc.robot.hawklibaries.drivetrains.DrivetrainConfig;
import frc.robot.hawklibaries.utilities.PIDConstants;

public class SwerveDrivetrainConfig extends DrivetrainConfig {

    private double[] moduleOffsets;
    private double moduleDistance;
    private PIDConstants translationPID;
    private PIDConstants rotationPID;

    private SwerveDrivetrainConfig(Builder builder) {
        super(builder);
        this.moduleOffsets = builder.moduleOffsets;
        this.moduleDistance = builder.moduleDistance;
        this.translationPID = builder.translationPID;
        this.rotationPID = builder.rotationPID;
    }

    public double[] getModuleOffsets() {return this.moduleOffsets;}
    public double getModuleDistance() {return this.moduleDistance;}
    public PIDConstants getTranslationPID() {return this.translationPID;}
    public PIDConstants getRotationPID() {return this.rotationPID;}

    public static class Builder extends DrivetrainConfig.Builder<Builder> {

        private double[] moduleOffsets;
        private double moduleDistance;
        private PIDConstants translationPID;
        private PIDConstants rotationPID;

        public Builder moduleOffsets(double[] moduleOffsets) {
            this.moduleOffsets = moduleOffsets;
            return this;
        }

        public Builder moduleDistance(double moduleDistance) {
            this.moduleDistance = moduleDistance;
            return this;
        }

        public Builder translationPID(PIDConstants translationPID) {
            this.translationPID = translationPID;
            return this;
        }

        public Builder rotationPID(PIDConstants rotationPID) {
            this.rotationPID = rotationPID;
            return this;
        }

        @Override
        protected Builder self() {
            return this;
        }

        @Override
        public SwerveDrivetrainConfig build() {
            return new SwerveDrivetrainConfig(this);
        }
    }
}