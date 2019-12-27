package raiderlib.path;

import java.util.ArrayList;

import raiderlib.geometry.Point;

/**
 * This abstract class is used as the base path that the robot follows To create
 * your own path, inherit this class and override the methods specified
 */
public abstract class Path {

    ArrayList<TrajPoint> points;
    ArrayList<WayPoint> waypoints;

    /**
     * Constructor for Path class
     */
    public Path() {
        waypoints = get_waypoints();
        calc_tan(waypoints);
        points = gen_points(waypoints);
        calc_curvature(points);
    }

    /**
     * This method is used for calculating the tangent vectors for each waypoint
     * 
     * @param waypoints Arraylist of waypoints
     */
    void calc_tan(final ArrayList<WayPoint> waypoints) {
        for (int i = 1; i < waypoints.size() - 1; i++)
            waypoints.get(i).tanPoint.copy(0.5 * (waypoints.get(i + 1).x - waypoints.get(i - 1).x),
                    0.5 * (waypoints.get(i + 1).y - waypoints.get(i - 1).y));
        findTan(waypoints.get(0), waypoints.get(1),
                Math.atan2(waypoints.get(0).tanPoint.y, waypoints.get(0).tanPoint.x), true);
        findTan(waypoints.get(waypoints.size() - 2), waypoints.get(waypoints.size() - 1), Math.atan2(
                waypoints.get(waypoints.size() - 1).tanPoint.y, waypoints.get(waypoints.size() - 1).tanPoint.x), false);

    }

    void findTan(WayPoint w1, WayPoint w2, double heading, boolean start) {
        double dist = 0;

        if (heading == 90 || heading == -90)
            dist = 2 * Math.abs(w1.y - w2.y);
        else if (heading == 0)
            dist = 2 * Math.abs(w1.x - w2.x);
        else {
            double m = Math.tan(heading * Math.PI / 180);
            double im = - 1 / m;
            if (start) {
                double x = (m * w2.x - im * w1.x + w1.y - w2.y) / (m - im);
                double y = m * (x - w2.x) + w2.y;
                dist = 2 * w2.dist(new Point(x, y));
            } else {
                double x = (m * w1.x - im * w2.x + w2.y - w1.y) / (m - im);
                double y = m * (x - w1.x) + w1.y;
                dist = 2 * w1.dist(new Point(x, y));
            }
        }
        if (start) {
            double x = w2.x - dist * Math.cos(heading);
            double y = w2.y - dist * Math.sin(heading);
            w1.tanPoint.x = (w2.x - x) * 0.5;
            w1.tanPoint.y = (w2.y - y) * 0.5;
        } else {
            double x = w1.x + dist * Math.cos(heading);
            double y = w1.y + dist * Math.sin(heading);
            w2.tanPoint.x = (x - w1.x) * 0.5;
            w2.tanPoint.y = (y - w1.y) * 0.5;
        }

    }

    /**
     * This method is used to calculate the curvature at each point
     * 
     * @param points Arraylist of points to calculate curvatures for
     */
    void calc_curvature(ArrayList<TrajPoint> points) {
        double E, D, F, h, k, r;
        for (int i = 1; i < points.size() - 1; i++) {
            E = ((points.get(i - 1).x - points.get(i).x)
                    * (points.get(i + 1).x * points.get(i + 1).x + points.get(i + 1).y * points.get(i + 1).y
                            - points.get(i - 1).x * points.get(i - 1).x - points.get(i - 1).y * points.get(i - 1).y)
                    - (points.get(i - 1).x - points.get(i + 1).x) * (points.get(i).x * points.get(i).x
                            + points.get(i).y * points.get(i).y - points.get(i - 1).x * points.get(i - 1).x
                            - points.get(i - 1).y * points.get(i - 1).y))
                    / ((points.get(i - 1).x - points.get(i + 1).x) * (points.get(i).y - points.get(i - 1).y)
                            - (points.get(i - 1).x - points.get(i).x) * (points.get(i + 1).y - points.get(i - 1).y));
            D = (points.get(i).x * points.get(i).x + points.get(i).y * points.get(i).y
                    - points.get(i - 1).x * points.get(i - 1).x - points.get(i - 1).y * points.get(i - 1).y
                    + E * points.get(i).y - E * points.get(i - 1).y) / (points.get(i - 1).x - points.get(i).x);
            F = -(points.get(i - 1).x * points.get(i - 1).x + points.get(i - 1).y * points.get(i - 1).y
                    + D * points.get(i - 1).x + E * points.get(i - 1).y);
            h = D / -2;
            k = E / -2;
            r = Math.sqrt(h * h + k * k - F);
            if (r == 0)
                points.get(i).curvature = 0;
            else
                points.get(i).curvature = 1 / r;
        }
    }

    /**
     * @return ArrayList of generated points from waypoints
     */
    ArrayList<TrajPoint> gen_points(ArrayList<WayPoint> waypoints) {
        ArrayList<TrajPoint> p = new ArrayList<TrajPoint>();
        p.add(new TrajPoint(waypoints.get(0).x, waypoints.get(0).y));
        for (int i = 1; i < waypoints.size(); i++) {
            PathSegment s = new PathSegment(waypoints.get(i - 1), waypoints.get(i));
            p.addAll(s.get_points());
        }
        return p;
    }

    /**
     * This method returns points generated from the path
     * 
     * @return trajectory points that form the path
     */
    public ArrayList<TrajPoint> get_points() {
        return this.points;
    }

    /**
     * This method returns a sublist of the points generated from the path
     * 
     * @param i starting index of sublist
     * @return trajectory points from the given index that form the path
     */
    public ArrayList<TrajPoint> get_points(int i) {
        return new ArrayList<TrajPoint>(this.points.subList(i, points.size() - 1));
    }

    /**
     * Override this method to input the waypoints of your path (default headings of
     * waypoints will be 0)
     * 
     * @return an Arraylist with the waypoints in your path
     */
    public abstract ArrayList<WayPoint> get_waypoints();
}