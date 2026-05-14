package frc.hawklibraries.mapZoner;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MapZoner {
    public HashMap<String, Zone> zoneMap;

    /**
     * Standard initialization with no preset values.
     * Please set values later using registerZone.
     */
    public MapZoner() {
        zoneMap = new HashMap<>();
    }

    /**
     * Adds a zone under a name stored as a string.
     * 
     * @param name The key of the zone to be called later.
     * @param inputZone The zone that us saved for use.
     */
    public void registerZone(String name, Zone inputZone) {
        zoneMap.put(name, inputZone);
    }

    /**
     * @param zoneName The name of the zone that was registered that is used for collision checking.
     * @param x The x cord to check if in zone.
     * @param y The y cord to check if in zone.
     * 
     * @return If the cords are in the zone.
     */
    public boolean check(String zoneName, double x, double y) {
        Zone zone = zoneMap.get(zoneName);

        return zone.getShape().contains(x, y);
    }

    /**
     * @param zoneName The name of the zone that was registered that is used for collision checking.
     * @param x The top left x cord of the rectangle.
     * @param y The top left y cord of the rectangle.
     * @param width The width of the rectangle.
     * @param height The height of the rectangle.
     * 
     * @return If the rectangle is in the zone.
     */
    public boolean check(String zoneName, double x, double y, double width, double height) {
        Zone zone = zoneMap.get(zoneName);

        return zone.getShape().contains(x, y, width, height);
    }

    /**
     * @param zoneName The name of the zone that was registered that is used for collision checking.
     * @param pos The position to check if in zone.
     * 
     * @return If the cords are in the zone.
     */
    public boolean check(String zoneName, Pose2d pos) {
        return check(zoneName, pos.getX(), pos.getY());
    }

    /**
     * @param zoneName The name of the zone that was registered that is used for collision checking.
     * @param pos The position to check if in zone.
     * @param width The width of the rectangle.
     * @param height The height of the rectangle.
     * 
     * @return If the rectangle is in the zone.
     */
    public boolean check(String zoneName, Pose2d pos, double width, double height) {
        return check(zoneName, pos.getX(), pos.getY(), width, height);
    }

    /**
     * @param zoneName The name of the zone that was registered that is used for collision checking.
     * @param pos The position to check if in zone.
     * 
     * @return If the cords are in the zone.
     */
    public boolean check(String zoneName, Translation2d pos) {
        return check(zoneName, pos.getX(), pos.getY());
    }

    /**
     * @param zoneName The name of the zone that was registered that is used for collision checking.
     * @param pos The position to check if in zone. Warning!!! This gets rid of angles.
     * @param width The width of the rectangle.
     * @param height The height of the rectangle.
     * 
     * @return If the rectangle is in the zone.
     */
    public boolean check(String zoneName, Translation2d pos, double width, double height) {
        return check(zoneName, pos.getX(), pos.getY(), width, height);
    }
}
