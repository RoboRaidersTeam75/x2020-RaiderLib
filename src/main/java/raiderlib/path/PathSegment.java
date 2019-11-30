package raiderlib.path;

import java.util.ArrayList;

/**
 * This class is used as a segment between two waypoints
 */
public class PathSegment{

    WayPoint startPoint;
    WayPoint endPoint;
    ArrayList<TrajPoint> points;
    /**
     * Constructor for PathSegment class
     * @param startPoint starting waypoint in segment
     * @param endPoint ending waypoint in segment
     */
    public PathSegment(final WayPoint startPoint, final WayPoint endPoint) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        points = gen_points();
    }

    /**
     * This method generates trajectory points in a segment
     * 
     * @return ArrayList of generated points
     */
    ArrayList<TrajPoint> gen_points() {
        final ArrayList<TrajPoint> p = new ArrayList<>();
        for (double i = 0.001; i <= 1; i += 0.001) {
            final double h1 = 2 * Math.pow(i, 3) - 3 * Math.pow(i, 2) + 1;
            final double h2 = -2 * Math.pow(i, 3) + 3 * Math.pow(i, 2);
            final double h3 = Math.pow(i, 3) - 2 * Math.pow(i, 2) + i;
            final double h4 = Math.pow(i, 3) - 2 * Math.pow(i, 2);
            final double x = h1 * this.startPoint.x + h2 * this.endPoint.x + h3 * this.startPoint.tanPoint.x
                    + h4 * this.endPoint.tanPoint.x;
            final double y = h1 * this.startPoint.y + h2 * this.endPoint.y + h3 * this.startPoint.tanPoint.y
                    + h4 * this.endPoint.tanPoint.y;
            p.add(new TrajPoint(x, y));
        }
        return p;
    }
    /**
     * This method returns the points in the segment
     * @return An ArrayList of the points in the segment
     */
    public ArrayList<TrajPoint> get_points(){
        return this.points;
    }
}