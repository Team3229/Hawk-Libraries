package frc.hawklibraries.utilities;

import edu.wpi.first.math.geometry.Pose2d;

public class Zone {
    private DrawType type;
    private double[] values;

    /*
     * How the zone would be drawn
     * Too lazy to do poly because of stuff
     */
    public static enum DrawType {
        Circle,
        Ellipse,
        CenterRectangle,
        TopLeftRectangle,
        Polygon
    }

    /*
     * Idk what to do with this it has no point
     */
    // public Zone() {

    // }

    public Zone(double x, double y, double width, double height, DrawType type) {
        setType(type);
        values = new double[4];

        values[0] = x;
        values[1] = y;
        values[2] = width;
        values[3] = height;
    }

    /*
     * Creates a zone in a circle shape
     */
    public Zone(double centerX, double centerY, double radius) {
        this(centerX, centerY, radius, radius, DrawType.Circle);
    }

    /*
     * Creates a zone in a circle shape but with a Pose2d
     */
    public Zone(Pose2d pos, double radius) {
        this(pos.getX(), pos.getY(), radius);
    }

    /*
     * Creates a zone in a center rectangle shape
     */
    public Zone(double centerX, double centerY, double width, double height) {
        this(centerX, centerY, width, height, DrawType.CenterRectangle);
    }

    /*
     * Creates a zone in a center rectangle shape but with a Pose2d
     */
    public Zone(Pose2d pos, double width, double height) {
        this(pos.getX(), pos.getY(), width, height);
    }

    public void setType(DrawType type) {
        this.type = type;
    }

    public DrawType getType() {
        return type;
    }

    public double[] getValues() {
        return values;
    }
}
