package raiderlib.path;

import raiderlib.geometry.Point;

/**
 * This class is used to represent a trajectory point in a path
 */
public class TrajPoint extends Point {
    public double curvature;
    public double velocity;

    /**
     * Constructor for TrajPoint class
     * 
     * @param x x coordinate
     * @param y y coordinate
     */
    public TrajPoint(final double x, final double y) {
        super(x, y);

    }

    /**
     * Constructor for TrajPoint class
     * 
     * @param x         x coordinate
     * @param y         y coordinate
     * @param curvature lookup curvature
     */
    public TrajPoint(final double x, final double y, final double curvature) {
        super(x, y);
        this.curvature = curvature;
    }

    /**
     * Constructor for TrajPoint class
     * 
     * @param x         x coordinate
     * @param y         y coordinate
     * @param curvature lookup curvature
     * @param vel       lookup velocity
     */
    public TrajPoint(final double x, final double y, final double curvature, final double vel) {
        super(x, y);
        this.curvature = curvature;
        this.velocity = vel;
    }

}