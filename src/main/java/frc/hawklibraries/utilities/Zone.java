package frc.hawklibraries.utilities;

public class Zone {
    private DrawType type;
    private double[] values;

    /*
     * How the zone would be drawn
     * Too lazy to do top left rec and poly because of alliance based stuff
     */
    public static enum DrawType {
        Circle,
        CenterRectangle,
        TopLeftRectangle,
        Polygon
    }

    /*
     * Idk what to do with this it has no point
     */
    public Zone() {

    }

    public Zone(double centerX, double centerY, double radius) {
        setType(DrawType.Circle);
    }

    public Zone(double centerX, double centerY, double width, double height) {
        setType(DrawType.CenterRectangle);
    }

    public Zone(double centerX, double centerY, double width, double height, DrawType type) {
        
    }

    public void setType(DrawType type) {
        this.type = type;
    }
}
