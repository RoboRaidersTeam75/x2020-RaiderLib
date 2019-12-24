package raiderlib.geometry;

/**
 * This class is used to represent a point in a cartesian plane
 */
public class Point {
    public double x;
    public double y;

    /**
     * Constructor for Point class
     * 
     * @param x x coordinate
     * @param y y coordinate
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * This method calculates distance from a given point
     * 
     * @param p point to calculate distance from
     * @return distance from point
     */
    public double dist(Point p) {
        return Math.sqrt(Math.pow(this.x - p.x, 2) + Math.pow(this.y - p.y, 2));
    }

    /**
     * This method copies a given point into the point
     * 
     * @param p point copied
     */
    public void copy(Point p) {
        copy(p.x, p.y);
    }

    /**
     * This method copies a given (x,y) into the point
     * 
     * @param x x coordinate copied
     * @param y y coordinate copied
     */
    public void copy(double x, double y) {
        this.x = x;
        this.y = y;
    }
}