package frc.hawklibraries.drivetrains.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleSimulator {
    private double currentAngle;
    private double currentSpeed;
    private double currentPosition;

    public SwerveModuleSimulator() {
        this.currentAngle = 0.0;
        this.currentSpeed = 0.0;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.currentAngle = desiredState.angle.getDegrees();
        this.currentSpeed = desiredState.speedMetersPerSecond;
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(currentSpeed, Rotation2d.fromDegrees(currentAngle));
    }

    public SwerveModulePosition toSwerveModulePosition() {
        return new SwerveModulePosition(currentPosition, Rotation2d.fromDegrees(currentAngle));
    }

    public void update(double deltaTime) {
        // Simulate module behavior over time if needed
        // For simplicity, this example assumes instantaneous response
        currentPosition += currentSpeed * deltaTime;
    }
}