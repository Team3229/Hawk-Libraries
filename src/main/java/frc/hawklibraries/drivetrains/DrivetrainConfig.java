/**
 * Configuration class for a drivetrain, encapsulating parameters such as speed, acceleration, and
 * robot dimensions. This class uses Lombok annotations for automatic generation of getters,
 * setters, and builders, simplifying configuration management.
 */
package frc.hawklibraries.drivetrains;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.SuperBuilder;

/**
 * Configuration for a drivetrain. <br>
 * This class provides a flexible way to define the operational limits and physical dimensions of a
 * robot's drivetrain, including speed, acceleration, turning rate, and the robot's size.
 */
@SuperBuilder
@Getter
@Setter
public class DrivetrainConfig {

  /**
   * Maximum speed of the drivetrain, in meters per second.<br>
   * <b>Default</b>: 3.0 meters/second.
   */
  @Builder.Default private LinearVelocity maxSpeed = MetersPerSecond.of(3.0);

  /**
   * Maximum acceleration of the drivetrain, in meters per second squared.<br>
   * <b>Default</b>: 1.0 meters/second^2.
   */
  @Builder.Default private LinearAcceleration maxAccel = MetersPerSecondPerSecond.of(1.0);

  /**
   * Maximum turning rate of the drivetrain, in radians per second.<br>
   * <b>Default</b>: 1.0 radians/second.
   */
  @Builder.Default private AngularVelocity maxTurnRate = RadiansPerSecond.of(1.0);

  /**
   * Width of the robot, in meters.<br>
   * <b>Default</b>: 1.0 meters.
   */
  @Builder.Default private Distance robotWidth = Meter.of(1.0);

  /**
   * Length of the robot, in meters.<br>
   * <b>Default</b>: 1.0 meters.
   */
  @Builder.Default private Distance robotLength = Meter.of(1.0);
}
