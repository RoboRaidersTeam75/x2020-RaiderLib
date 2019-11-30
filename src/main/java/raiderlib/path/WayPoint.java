package raiderlib.path;

import raiderlib.geometry.Point;
/**
 * This class is used to represent a waypoint in a path
 */
public class WayPoint extends Point{

    Point tanPoint;
    /**
     * Constructor for WayPoint class
     * @param x x coordinate 
     * @param y y coordinate
     * @param tanPoint tangent point used in splining
     */
	public WayPoint(double x, double y, Point tanPoint) {
        super(x, y);
        this.tanPoint = tanPoint;
	}

}