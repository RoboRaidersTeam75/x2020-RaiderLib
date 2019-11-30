package raiderlib.geometry;
/**
 * This class is used to represent a position on field
 */
public class Pose extends Point{
    public double theta;
    /**
     * Constructor for Pose class
     * @param x x coordinate
     * @param y y coordinate
     * @param theta heading in radians
     */
    public Pose(final double x, final double y, final double theta) {
        super(x, y);
        this.theta = theta;
    }

    /**
     * This method is used to add a given pose
     * 
     * @param p pose object added
     * @return sum of poses
     */
    public Pose add_point(final Pose p) {
        this.x += p.x;
        this.y += p.y;
        this.theta += p.theta;
        return this;
    }

}