package frc.hawklibraries.utilities;

import java.util.HashMap;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Ellipse2D;

import frc.hawklibraries.utilities.Zone.DrawType;

public class MapZoner {
    private HashMap<String, Zone> zoneMap;

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
        double[] zoneValues = zone.getValues();

        if (zone.getType().equals(DrawType.CenterRectangle)) {
            return new Rectangle2D.Double(zoneValues[0], zoneValues[1], zoneValues[2], zoneValues[3]).contains(botX, botY);
        } else if (zone.getType().equals(DrawType.TopLeftRectangle)) {
            return new Rectangle2D.Double(zoneValues[0] + zoneValues[2] / 2, zoneValues[1] + zoneValues[3] / 2, zoneValues[2] * 2, zoneValues[3] * 2).contains(botX, botY);
        } else if (zone.getType().equals(DrawType.Circle)) {
            return new Ellipse2D.Double(zoneValues[0], zoneValues[1], zoneValues[2], zoneValues[3]).contains(botX, botY);
        } else if (zone.getType().equals(DrawType.Ellipse)) {
            return new Ellipse2D.Double(zoneValues[0], zoneValues[1], zoneValues[2], zoneValues[3]).contains(botX, botY);
        }

        return false;
    }
}
