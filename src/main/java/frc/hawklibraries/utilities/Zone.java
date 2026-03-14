package frc.hawklibraries.utilities;

import java.awt.Shape;
import java.awt.Polygon;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Area;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

// note: make the VZ/HZ the game board width/height

/**
 * Used to store shape data in a easy-to-learn way 
 * Does not support angles
 * Suggestion: use top left ... 
 */
public class Zone {
    private Shape shapeHolder;

    /**
     * How a zone would be drawn
     * Too lazy to do directional zones because of questionable implementation
     */
    public static enum DrawType {
        Circle,
        CenterEllipse,
        TopLeftEllipse,
        CenterRectangle,
        TopLeftRectangle,
        TwoPointRectangle,
        Polygon,
        VerticalZone,
        HorizontalZone
    }

    /**
     * Universal constructor
     * 
     * Excludes polygons
     */
    public Zone(double x, double y, double width, double height, DrawType type) {
        if (type.equals(DrawType.CenterRectangle)) {
            // Width and height are radius style.
            shapeHolder = new Rectangle2D.Double(x - width / 2, y - height / 2, width / 2, height / 2);
        } else if (type.equals(DrawType.TopLeftRectangle)) {
            shapeHolder = new Rectangle2D.Double(x, y, width, height);
        } else if (type.equals(DrawType.Circle) || type.equals(DrawType.CenterEllipse)) {
            // Width and height are radius style. I hope
            shapeHolder = new Ellipse2D.Double(x - width / 2, y - height / 2, width / 2, height / 2);
        } else if (type.equals(DrawType.TopLeftEllipse)) {
            shapeHolder = new Rectangle2D.Double(x, y, width, height);
        } else if (type.equals(DrawType.TwoPointRectangle)) {
            // width/height is treated as x2, y2 
            shapeHolder = new Rectangle2D.Double(x, y, width - x, height - y);
        }
    }

    /**
     * Mimic of the standard contructor but using wpi classes
     */
    public Zone(Pose2d pos, double width, double height, DrawType type) {
        this(pos.getX(), pos.getY(), width, height, type);
    }

    /**
     * Mimic of the standard contructor but using wpi classes
     * Warning!!! This gets rid of angles.
     */
    public Zone(Translation2d pos, double width, double height, DrawType type) {
        this(pos.getX(), pos.getY(), width, height, type);
    }

    // Polygon

    /**
     * Polygon constructor
     * 
     * Loses point percision as it goes from double -> int. aka round(double)
     * 
     * @param points Holds points in the format of [x1, y1, x2, y2, ...]
     */
    public Zone(double[] points) {
        int[] x = new int[points.length / 2];
        int[] y = new int[points.length / 2];

        for (int i = 0; i < points.length / 2; i++) {
            x[i] = (int) Math.round(points[i * 2]);
            y[i] = (int) Math.round(points[i * 2 + 1]);
        }

        shapeHolder = new Area(new Polygon(x, y, x.length));
    }

    /**
     * Polygon constructor
     * 
     * Loses point percision as it goes from double -> int. aka round(double)
     * 
     * @param points Holds points in the format of [[x1, y1], [x2, y2], ...]
     */
    public Zone(double[][] points) {
        int[] x = new int[points.length];
        int[] y = new int[points.length];

        for (int i = 0; i < points.length; i++) {
            x[i] = (int) Math.round(points[i][0]);
            y[i] = (int) Math.round(points[i][1]);
        }

        shapeHolder = new Area(new Polygon(x, y, x.length));
    }

    /**
     * Polygon constructor
     * 
     * Loses point percision as it goes from double -> int. aka round(double)
     * 
     * @param points An array of Pose2d
     */
    public Zone(Pose2d[] points) {
        int[] x = new int[points.length / 2];
        int[] y = new int[points.length / 2];

        for (int i = 0; i < points.length; i++) {
            x[i] = (int) Math.round(points[i].getX());
            y[i] = (int) Math.round(points[i].getY());
        }

        shapeHolder = new Area(new Polygon(x, y, x.length));
    }

    /**
     * Polygon constructor
     * 
     * Loses point percision as it goes from double -> int. aka round(double)
     * No angles 
     * 
     * @param points An array of Translation2d
     */
    public Zone(Translation2d[] points) {
        int[] x = new int[points.length / 2];
        int[] y = new int[points.length / 2];

        for (int i = 0; i < points.length; i++) {
            x[i] = (int) Math.round(points[i].getX());
            y[i] = (int) Math.round(points[i].getY());
        }

        shapeHolder = new Area(new Polygon(x, y, x.length));
    }

    // Circles

    /**
     * Creates a zone in a circle shape
     */
    public Zone(double centerX, double centerY, double radius) {
        this(centerX, centerY, radius, radius, DrawType.Circle);
    }

    /**
     * Creates a zone in a circle shape but with a Pose2d
     */
    public Zone(Pose2d pos, double radius) {
        this(pos.getX(), pos.getY(), radius);
    }

    /**
     * Creates a zone in a circle shape but with a Translation2d
     * Warning!!! This gets rid of angles.
     */
    public Zone(Translation2d pos, double radius) {
        this(pos.getX(), pos.getY(), radius);
    }

    // Top left rec

    /**
     * Creates a zone in a center rectangle shape
     */
    public Zone(double x, double y, double width, double height) {
        this(x, y, width, height, DrawType.TopLeftRectangle);
    }

    /**
     * Creates a zone in a top left rectangle shape but with a Pose2d
     */
    public Zone(Pose2d pos, double width, double height) {
        this(pos.getX(), pos.getY(), width, height);
    }

    /**
     * Creates a zone in a top left rectangle shape but with a Translation2d
     * Warning!!! This gets rid of angles.
     */
    public Zone(Translation2d pos, double width, double height) {
        this(pos.getX(), pos.getY(), width, height);
    }

    // 2 point rec

    /**
     * Creates a rectangle with the first point being top left and the second being bottom right
     * 
     * @param point1 double array with index 0 being x and index 1 being y
     * @param point2 double array with index 0 being x and index 1 being y
     */
    public Zone(double[] point1, double[] point2) {
        this(point1[0], point1[1], point2[0], point2[1], DrawType.TopLeftRectangle);
    }

    /**
     * Creates a rectangle with the first point being top left and the second being bottom right
     * 
     * @param point1 A Pose2d for the top left
     * @param point2 A Pose2d for the bottom right
     */
    public Zone(Pose2d point1, Pose2d point2) {
        this(
            new double[] { point1.getX(), point1.getY() },
            new double[] { point2.getX(), point2.getY() }
        );
    }

    /**
     * Creates a rectangle with the first point being top left and the second being bottom right
     * 
     * @param point1 A Translation2d for the top left
     * @param point2 A Translation2d for the bottom right
     */
    public Zone(Translation2d point1, Translation2d point2) {
        this(
            new double[] { point1.getX(), point1.getY() },
            new double[] { point2.getX(), point2.getY() }
        );
    }

    public Shape getShape() {
        return shapeHolder;
    }
}
