package frc.hawklibraries.utilities;

import java.util.HashMap;

public class MapZoner {
    private HashMap<String, Zone> zoneMap;

    /*
     * Standard initialization with no preset values
     * Please set values later using <put stuff here>
     */
    public MapZoner() {
        zoneMap = new HashMap<>();
    }

    public void registerZone(Zone inputZone, String name) {
        zoneMap.put(name, inputZone);
    }

    public boolean check(String zoneName, double botX, double botY) {
        

        return false;
    }

    private void updateZone(double botX, double botY) {

    }
}
