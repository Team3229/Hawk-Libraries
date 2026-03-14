package frc.hawklibraries.utilities;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MapZoner {
    public HashMap<String, Zone> zoneMap;

    /**
     * Standard initialization with no preset values
     * Please set values later using registerZone
     */
    public MapZoner() {
        zoneMap = new HashMap<>();
    }

    /**
     * Adds a zone under a name stored as a string
     */
    public void registerZone(String name, Zone inputZone) {
        zoneMap.put(name, inputZone);
    }

    /**
     * Returns if the x and y cords are in a registered zone
     */
    public boolean check(String zoneName, double botX, double botY) {
        Zone zone = zoneMap.get(zoneName);

        return zone.getShape().contains(botX, botY);
    }

    /**
     * Returns if the rectangle is in a registered zone
     */
    public boolean check(String zoneName, double botX, double botY, double width, double height) {
        Zone zone = zoneMap.get(zoneName);

        return zone.getShape().contains(botX, botY, width, height);
    }

    /**
     * Returns if the x and y cords are in a registered zone
     */
    public boolean check(String zoneName, Pose2d pos) {
        return check(zoneName, pos.getX(), pos.getY());
    }

    /**
     * Returns if the rectangle is in a registered zone
     */
    public boolean check(String zoneName, Pose2d pos, double width, double height) {
        return check(zoneName, pos.getX(), pos.getY(), width, height);
    }

    /**
     * Returns if the x and y cords are in a registered zone
     * Warning!!! This gets rid of angles.
     */
    public boolean check(String zoneName, Translation2d pos) {
        return check(zoneName, pos.getX(), pos.getY());
    }

    /**
     * Returns if  the rectangle is in a registered zone
     * Warning!!! This gets rid of angles.
     */
    public boolean check(String zoneName, Translation2d pos, double width, double height) {
        return check(zoneName, pos.getX(), pos.getY(), width, height);
    }
}
