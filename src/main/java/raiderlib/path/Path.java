package raiderlib.path;

import java.util.ArrayList;

import raiderlib.control.DriveCharacterization;
/**
 * This abstract class is used as the base path that the robot follows
 * To create your own path, inherit this class and override the methods specified
 */
public abstract class Path {

    ArrayList<TrajPoint> points;
    ArrayList<WayPoint> waypoints;

    DriveCharacterization driveCharacterization;

    int prevLookAheadPoint = 0;
    int prevClosestPoint = 0;
    /**
     * Constructor for Path class
     */
    public Path() {
        waypoints = get_waypoints();
        driveCharacterization = get_drive_characterization();
        calc_tan(waypoints);
        points = gen_points(waypoints);
        calc_velocity(driveCharacterization, points);
    }
    /**
     * This method is used for calculating the tangent vectors for each waypoint
     * @param waypoints Arraylist of waypoints
     */
    void calc_tan(final ArrayList<WayPoint> waypoints) {
        for (int i = 1; i < waypoints.size() - 2; i++)
            waypoints.get(i).copy(0.5 * (waypoints.get(i + 1).x - waypoints.get(i - 1).x),
                    0.5 * (waypoints.get(i + 1).y - waypoints.get(i - 1).y));
    }
    /**
     * This method is used to calculate the lookup velocites at each trajectory point
     * @param driveCharacterization characterization of the drivetrain
     * @param points arraylist of points to set velocites for
     */
    void calc_velocity(DriveCharacterization driveCharacterization, ArrayList<TrajPoint> points) {
        calc_curvature(points);
        for (int i = 1; i < points.size() - 2; i++) {
            if (1 / points.get(i).curvature < driveCharacterization.maxVelocity)
                points.get(i).velocity = 1 / points.get(i).curvature;
            else
                points.get(i).velocity = driveCharacterization.maxVelocity;
        }
        points.get(0).velocity = 0;
        points.get(points.size() - 1).velocity = 0;
        double plausVel = 0;
        for (int i = 1; i < points.size() - 2; i++) {
            plausVel = Math.sqrt(Math.pow(points.get(i - 1).velocity, 2)
                    + 2 * driveCharacterization.maxAcceleration * points.get(i).dist(points.get(i - 1)));
            if (plausVel < points.get(i).velocity)
                points.get(i).velocity = plausVel;
        }
        for (int i = points.size() - 2; i > 0; i--) {
            plausVel = Math.sqrt(Math.pow(points.get(i + 1).velocity, 2)
                    + 2 * driveCharacterization.maxAcceleration * points.get(i).dist(points.get(i + 1)));
            if (plausVel < points.get(i).velocity)
                points.get(i).velocity = plausVel;
        }
    }
    /**
     * This method is used to calculate the curvature at each point
     * @param points Arraylist of points to calculate curvatures for
     */
    void calc_curvature(ArrayList<TrajPoint> points) {
        double E, D, F, h, k, r;
        for (int i = 1; i < points.size() - 2; i++) {
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
                r += 0.000000000000000000000000000000000000001;
            points.get(i).curvature = 1 / r;
        }
    }

    /**
     * @return ArrayList of generated points from waypoints
     */
    ArrayList<TrajPoint> gen_points(final ArrayList<WayPoint> waypoints) {
        final ArrayList<TrajPoint> p = new ArrayList<>();
        for (int i = 1; i < waypoints.size() - 1; i++) {
            final PathSegment s = new PathSegment(waypoints.get(i - 1), waypoints.get(i));
            p.addAll(s.get_points());
        }
        return p;
    }
    /**
     * This method returns points generated from the path
     * @return trajectory points that form the path
     */
    public ArrayList<TrajPoint> get_points(){
        return this.points;
    }
    /**
     * This method returns a sublist of the points generated from the path
     * @param i starting index of sublist
     * @return trajectory points from the given index that form the path
     */
    public ArrayList<TrajPoint> get_points(int i){
        return (ArrayList<TrajPoint>)this.points.subList(i, points.size() - 1);
    }
    /**
     * 
     * @return previous lookahead point ID
     */
    public int getLookAheadID(){
        return this.prevLookAheadPoint;
    }
    /**
     * 
     * @return previous closest point ID
     */
    public int getClosestPointID(){
        return this.prevClosestPoint;
    }
    /**
     * This method sets the lookAheadPointID
     * @param ID index of lookAheadPoint
     * @return returns whether the ID was valid
     */
    public boolean setLookAheadID(int ID){
        if(ID > this.points.size() - 1)
            return false;
        this.prevLookAheadPoint = ID;
        return true;
    }
    /**
     * This method sets the closestPointtID
     * @param ID index of closestPoint
     * @return returns whether the ID was valid
     */
    public boolean setClosestPointID(int ID){
        if(ID > this.points.size() - 1)
            return false;
        this.prevClosestPoint = ID;
        return true;
    }
    /**
     * Override this method to input the drive characterization
     * @return drive characterization of robot
     */       
    public abstract DriveCharacterization get_drive_characterization();
    /**
     * Override this method to input the waypoints of your path (default headings of waypoints will be 0)
     * @return an Arraylist with the waypoints in your path
     */
    public abstract ArrayList<WayPoint> get_waypoints();
}