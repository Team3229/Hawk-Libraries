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
    // public Zone() {

    // }

    public Zone(double centerX, double centerY, double radius) {
        setType(DrawType.Circle);
        
        values = new double[3];
        values[0] = centerX;
        values[1] = centerY;
        values[2] = radius;
    }

    public Zone(double centerX, double centerY, double width, double height) {
        setType(DrawType.CenterRectangle);

        values = new double[4];
        values[0] = centerX;
        values[1] = centerY;
        values[2] = width;
        values[3] = height;
    }

    public Zone(double x, double y, double width, double height, DrawType recType) {
        setType(recType);
        values = new double[4];

        values[0] = x;
        values[1] = y;
        values[2] = width;
        values[3] = height;
    }

    public void setType(DrawType type) {
        this.type = type;
    }
}
