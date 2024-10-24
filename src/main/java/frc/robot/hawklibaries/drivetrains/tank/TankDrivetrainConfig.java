package frc.robot.hawklibaries.drivetrains.tank;

import frc.robot.hawklibaries.drivetrains.DrivetrainConfig;

public class TankDrivetrainConfig extends DrivetrainConfig {

    private double trackWidth;

    private TankDrivetrainConfig(Builder builder) {
        super(builder);

        this.trackWidth = builder.trackWidth;
    }

    public double getTrackWidth() {return this.trackWidth;}

    public static class Builder extends DrivetrainConfig.Builder<Builder> {

        private double trackWidth = 5;

        public Builder trackWidth(double trackWidth) {
            this.trackWidth = trackWidth;
            return self();
        }

        @Override
        protected Builder self() {
            return this;
        }

        @Override
        public TankDrivetrainConfig build() {
            return new TankDrivetrainConfig(this);
        }
    }
}