package frc.hawklibraries.utilities;

import java.util.HashMap;

public class MapZoner {
    public HashMap<String, Zone> zoneMap;

    /*
     * Standard initialization with no preset values
     * Please set values later using registerZone
     */
    public MapZoner() {
        zoneMap = new HashMap<>();
    }

    public void registerZone(String name, Zone inputZone) {
        zoneMap.put(name, inputZone);
    }

    /*
     * Returns if the x and y cords are in a registered zone
     */
    public boolean check(String zoneName, double botX, double botY) {
        Zone zone = zoneMap.get(zoneName);

        return zone.getShape().contains(botX, botY);
    }
}
