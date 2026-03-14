package frc.hawklibraries.utilities;

import java.awt.Shape;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Ellipse2D;

public class Zone {
    private Shape shapeHolder;

    /**
     * How the zone would be drawn
     * Too lazy to do poly because of stuff
     * 
     * Circle is kinda useless
     */
    public static enum DrawType {
        Circle,
        Ellipse,
        CenterRectangle,
        TopLeftRectangle,
        Polygon,
        VerticalZone,
        HorizontalZone
    }

    /**
     * Universal constructors
     */
    public Zone(double x, double y, double width, double height, DrawType type) {

        if (type.equals(DrawType.CenterRectangle)) {
            shapeHolder = new Rectangle2D.Double(x, y, width, height);
        } else if (type.equals(DrawType.TopLeftRectangle)) {
            shapeHolder = new Rectangle2D.Double(x + width / 2, y + height / 2, width * 2, height * 2);
        } else if (type.equals(DrawType.Circle) || type.equals(DrawType.Ellipse)) {
            shapeHolder = new Ellipse2D.Double(x, y, width, height);
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

    /**
     * Creates a zone in a center rectangle shape
     */
    public Zone(double centerX, double centerY, double width, double height) {
        this(centerX, centerY, width, height, DrawType.CenterRectangle);
    }

    /**
     * Creates a zone in a center rectangle shape but with a Pose2d
     */
    public Zone(Pose2d pos, double width, double height) {
        this(pos.getX(), pos.getY(), width, height);
    }

    /**
     * Creates a zone in a center rectangle shape but with a Translation2d
     * Warning!!! This gets rid of angles.
     */
    public Zone(Translation2d pos, double width, double height) {
        this(pos.getX(), pos.getY(), width, height);
    }

    public Shape getShape() {
        return shapeHolder;
    }
}
