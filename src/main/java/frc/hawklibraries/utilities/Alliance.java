package frc.hawklibraries.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.NoSuchElementException;

/**
 * Utility class for determining the alliance color of the robot. This class provides methods to get
 * the current alliance color and to create triggers based on alliance presence.
 */
public class Alliance {

  /** Represents the color of an alliance in a competition. */
  public static enum AllianceColor {
    /** Represents the red alliance. */
    Red,
    /** Represents the blue alliance. */
    Blue,
    /** Represents a null or undefined alliance. */
    Null
  }

  /**
   * Gets the current alliance color of the robot.
   *
   * @return Current alliance color.
   */
  public static AllianceColor getAlliance() {
    try {
      return wpiToHawk(DriverStation.getAlliance().get());
    } catch (NoSuchElementException e) {
      return AllianceColor.Null;
    }
  }

  /**
   * Creates a trigger that activates when the alliance is present.
   *
   * @return Trigger that activates when the alliance is present.
   */
  public static Trigger alliancePresentTrigger() {
    return new Trigger(() -> DriverStation.getAlliance().isPresent());
  }

  /**
   * Converts WPILib alliance enum to Hawk alliance enum.
   *
   * @param a WPILib alliance enum.
   * @return Hawk alliance enum.
   */
  private static AllianceColor wpiToHawk(edu.wpi.first.wpilibj.DriverStation.Alliance a) {
    if (a == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
      return AllianceColor.Blue;
    } else {
      return AllianceColor.Red;
    }
  }
}
