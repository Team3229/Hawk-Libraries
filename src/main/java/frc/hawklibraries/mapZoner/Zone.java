package frc.hawklibraries.mapZoner;

import java.awt.Shape;
import java.awt.Polygon;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Area;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Used to store shape data in a easy-to-learn way. Used best with MapZoner. 
 * Does not support angles. 
 * Suggestion: use top left DrawTypes. 
 */
public class Zone {
    private Shape shapeHolder;

    /**
     * How a zone would be drawn. 
     */
    public static enum DrawType {
        Circle,
        CenterEllipse,
        TopLeftEllipse,
        CenterRectangle,
        TopLeftRectangle,
        TwoPointRectangle,
        Polygon
    }

    /**
     * Universal constructor.
     * Excludes polygons.
     * 
     * @param x The starting x cord of the shape.
     * @param y The starting y cord of the shape.
     * @param x2 The first value (usually but not all the time width).
     * @param y2 The second value (usually but not all the time height).
     * @param type The type of shape to draw (view DrawType for the types).
     */
    public Zone(double x, double y, double x2, double y2, DrawType type) {
        if (type.equals(DrawType.CenterRectangle)) {
            // Width and height are radius style.
            shapeHolder = new Rectangle2D.Double(x - x2 / 2, y - y2 / 2, x2 / 2, y2 / 2);
        } else if (type.equals(DrawType.TopLeftRectangle)) {
            shapeHolder = new Rectangle2D.Double(x, y, x2, y2);
        } else if (type.equals(DrawType.Circle) || type.equals(DrawType.CenterEllipse)) {
            // Width and height are radius style. I hope
            shapeHolder = new Ellipse2D.Double(x - x2 / 2, y - y2 / 2, x2 / 2, y2 / 2);
        } else if (type.equals(DrawType.TopLeftEllipse)) {
            shapeHolder = new Rectangle2D.Double(x, y, x2, y2);
        } else if (type.equals(DrawType.TwoPointRectangle)) {
            // width/height is treated as x2, y2 
            shapeHolder = new Rectangle2D.Double(x, y, x2 - x, y2 - y);
        }
    }

    /**
     * Mimic of the standard contructor but using a {@code Pose2d}.
     * 
     * @param pos The starting position of the shape.
     * @param x2 The first value (usually but not all the time width).
     * @param y2 The second value (usually but not all the time height).
     * @param type The type of shape to draw (view DrawType for the types).
     */
    public Zone(Pose2d pos, double x2, double y2, DrawType type) {
        this(pos.getX(), pos.getY(), x2, y2, type);
    }

    /**
     * Mimic of the standard contructor but using wpi {@code Translation2d}.
     * 
     * @param pos The starting position of the shape. Warning!!! This gets rid of angles.
     * @param x2 The first value (usually but not all the time width).
     * @param y2 The second value (usually but not all the time height).
     * @param type The type of shape to draw (view DrawType for the types).
     */
    public Zone(Translation2d pos, double x2, double y2, DrawType type) {
        this(pos.getX(), pos.getY(), x2, y2, type);
    }

    // Polygon

    /**
     * Polygon constructor. 
     * 
     * Loses point percision as it goes from double -> int.
     * 
     * @param points Holds points in the format of {@code [x1, y1, x2, y2, ...]}.
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
     * Polygon constructor.
     * 
     * Loses point percision as it goes from double -> int.
     * 
     * @param points Holds points in the format of {@code [[x1, y1], [x2, y2], ...]}.
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
     * Polygon constructor.
     * 
     * Loses point percision as it goes from double -> int.
     * 
     * @param points An array of Pose2d used to map the polygon.
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
     * Polygon constructor.
     * 
     * Loses point percision as it goes from double -> int.
     * 
     * @param points An array of Translation2d used to map the polygon. Warning!!! Does not use angles.
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
     * Creates a zone in a circle shape.
     * 
     * @param x The center x cord of the circle.
     * @param y The center y cord of the circle.
     * @param radius The radius of the circle.
     */
    public Zone(double x, double y, double radius) {
        this(x, y, radius, radius, DrawType.Circle);
    }

    /**
     * Creates a zone in a circle shape but with a {@code Pose2d}.
     * 
     * @param pos The center of the circle.
     * @param radius The radius of the circle.
     */
    public Zone(Pose2d pos, double radius) {
        this(pos.getX(), pos.getY(), radius);
    }

    /**
     * Creates a zone in a circle shape but with a {@code Translation2d}.
     * 
     * @param pos The center of the circle. Warning!!! This gets rid of angles.
     * @param radius The radius of the circle.
     */
    public Zone(Translation2d pos, double radius) {
        this(pos.getX(), pos.getY(), radius);
    }

    // Top left rec

    /**
     * Creates a zone in a top left rectangle shape.
     * 
     * @param x The x cord of the rectangle.
     * @param y The y cord of the rectangle.
     * @param width The width of the rectangle.
     * @param height The height of the rectangle.
     */
    public Zone(double x, double y, double width, double height) {
        this(x, y, width, height, DrawType.TopLeftRectangle);
    }

    /**
     * Creates a zone in a top left rectangle shape but with a {@code Pose2d}.
     * 
     * @param pos The position of the rectangle.
     * @param width The width of the rectangle.
     * @param height The height of the rectangle.
     */
    public Zone(Pose2d pos, double width, double height) {
        this(pos.getX(), pos.getY(), width, height);
    }

    /**
     * Creates a zone in a top left rectangle shape but with a {@code Translation2d}.
     * 
     * @param pos The position of the rectangle. Warning!!! This gets rid of angles.
     * @param width The width of the rectangle.
     * @param height The height of the rectangle.
     */
    public Zone(Translation2d pos, double width, double height) {
        this(pos.getX(), pos.getY(), width, height);
    }

    // 2 point rec

    /**
     * Creates a rectangle with the first point being top left and the second being bottom right.
     * 
     * @param point1 An array with the format of {@code [x1, y1]}.
     * @param point2 An array with the format of {@code [x2, y2]}.
     */
    public Zone(double[] point1, double[] point2) {
        this(point1[0], point1[1], point2[0], point2[1], DrawType.TopLeftRectangle);
    }

    /**
     * Creates a rectangle with the first point being top left and the second being bottom right.
     * 
     * @param point1 A Pose2d for the top left.
     * @param point2 A Pose2d for the bottom right.
     */
    public Zone(Pose2d point1, Pose2d point2) {
        this(
            new double[] { point1.getX(), point1.getY() },
            new double[] { point2.getX(), point2.getY() }
        );
    }

    /**
     * Creates a rectangle with the first point being top left and the second being bottom right.
     * 
     * @param point1 A Translation2d for the top left.
     * @param point2 A Translation2d for the bottom right.
     */
    public Zone(Translation2d point1, Translation2d point2) {
        this(
            new double[] { point1.getX(), point1.getY() },
            new double[] { point2.getX(), point2.getY() }
        );
    }

    /**
     * @return The shape of the zone.
     */
    public Shape getShape() {
        return shapeHolder;
    }
}
