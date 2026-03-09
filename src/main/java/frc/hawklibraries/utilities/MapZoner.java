package frc.hawklibraries.utilities;

import java.util.HashMap;

public class MapZoner {
    private HashMap<String, Boolean> zoneMap;

    /*
     * Standard initialization with no preset values
     * Please set values later using <put stuff here>
     */
    public MapZoner() {
        zoneMap = new HashMap<String, Boolean>();
    }

    /*
     * Bit funky don't use
     * Allows a input of arrays that should contain the following format, [top left x, top left y, bottom right x, bottom right y]
     */
    public MapZoner(double[][] presetMap) {

    }
}
