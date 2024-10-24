package frc.robot.hawklibaries.drivetrains.tank;

import frc.robot.hawklibaries.drivetrains.DrivetrainConfig;

public class TankDrivetrainConfig extends DrivetrainConfig {

    private TankDrivetrainConfig(Builder builder) {
        super(builder);
    }

    public static class Builder extends DrivetrainConfig.Builder<Builder> {

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