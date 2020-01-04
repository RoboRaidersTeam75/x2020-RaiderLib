package raiderlib.control;

import java.util.ArrayList;

import raiderlib.geometry.Point;
import raiderlib.geometry.Pose;
import raiderlib.path.Path;
import raiderlib.path.TrajPoint;

/**
 * This class is used as a PurePursuitController. To pursuit a path, use the
 * pursuit_path method To pursuit a point, use the pursuit_point method
 */
public class PurePursuitController {
    DriveCharacterization driveCharacterization;
    double lookAheadDistance;
    boolean isFinished;
    Path path;
    int lookAheadPoint = 0;
    int closestPoint = 0;

    /**
     * Constructor for PurePursuitController class
     * 
     * @param driveCharacterization driveCharacterization of robot
     * @param lookAheadDistance     max lookahead distance
     */
    public PurePursuitController(final Path path, final DriveCharacterization driveCharacterization,
            final double lookAheadDistance) {
        this.driveCharacterization = driveCharacterization;
        this.lookAheadDistance = lookAheadDistance;
        this.path = path;
        calc_velocity(driveCharacterization, path.get_points());
        isFinished = false;
    }

    public PurePursuitController(final DriveCharacterization driveCharacterization, final double lookAheadDistance) {
        this.driveCharacterization = driveCharacterization;
        this.lookAheadDistance = lookAheadDistance;
        isFinished = false;
    }

    /**
     * This method is used to pursuit a path
     * 
     * @param path     path to be pursuited
     * @param currPose robot's current pose
     * @return a DriveCommand for the robot to follow
     */
    public DriveCommand pursuit_path(final Pose currPose) {
        final TrajPoint closestPoint = get_closest_point(path, currPose);
        final Point lookAheadPoint = get_lookAhead_point(path, currPose,
                (1 / closestPoint.curvature < this.lookAheadDistance) ? 1 / closestPoint.curvature
                        : this.lookAheadDistance);
        if (Math.abs(currPose.dist(lookAheadPoint)) <= 1) {
            isFinished = true;
            return new DriveCommand(0, 0);
        }
        return pursuit_point(lookAheadPoint, currPose, closestPoint.velocity);
    }

    /**
     * This method tells whether the robot has finished the path
     * 
     * @return a boolean of whether the robot has finished the path
     */
    public boolean is_finished() {
        return isFinished;
    }

    /**
     * This method is used to pursuit a point
     * 
     * @param p        point to pursuit
     * @param currPose robot's current pose
     * @param velocity velocity to pursuit at
     * @return a DriveCommand for the robot to follow
     */
    public DriveCommand pursuit_point(final Point p, final Pose currPose, final double velocity) {
        final double arcCurve = get_arc_curvature(p, currPose);
        return new DriveCommand(velocity * (2 + arcCurve * this.driveCharacterization.trackWidth),
                velocity * (2 - arcCurve * this.driveCharacterization.trackWidth));
    }
    /**
     * This method resets the lookahead IDs and lookup velocites
     */
    public void reset(){
        lookAheadPoint = 0;
        closestPoint = 0;
        isFinished = false;
    }
    /**
     * This method is used to calculate the curvature of the arc to a lookahead
     * point
     * 
     * @param lookAheadPoint the lookahead point
     * @param currPose       robot's current pose
     * @return the curvature of the arc
     */
    double get_arc_curvature(final Point lookAheadPoint, final Pose currPose) {
        final double a = -Math.tan(currPose.theta);
        final double c = Math.tan(currPose.theta) * currPose.x - currPose.y;
        final double x = Math.abs(a * lookAheadPoint.x + lookAheadPoint.y + c) / Math.sqrt(a * a + 1);
        final int sign = ((Math.sin(currPose.theta) * (lookAheadPoint.x - currPose.x)
                - Math.cos(currPose.theta) * (lookAheadPoint.y - currPose.y)) > 0) ? 1 : -1;
        return sign * 2 * x / (Math.pow(currPose.dist(lookAheadPoint), 2));
    }

    /**
     * This method is used to find the lookahead point on a path
     * 
     * @param path              the path the robot follows
     * @param currPose          robot's current pose
     * @param lookAheadDistance lookahead distance of robot
     * @return the point on the path within the lookahead radius. If there is more
     *         than one, the robot chooses the later one on the path
     */
    TrajPoint get_lookAhead_point(final Path path, final Pose currPose, final double lookAheadDistance) {
        int ID = lookAheadPoint;
        final ArrayList<TrajPoint> points = path.get_points(ID);
        TrajPoint output = points.get(0);
        for (final TrajPoint p : points) {
            if ((Math.pow(p.x - currPose.x, 2) + Math.pow(p.y - currPose.y, 2)) <= Math.pow(lookAheadDistance, 2) + 5
                    && (Math.pow(p.x - currPose.x, 2) + Math.pow(p.y - currPose.y, 2)) >= Math.pow(lookAheadDistance, 2)
                            - 5) {
                lookAheadPoint = ID;
                output = p;
            }
            ID++;
        }
        return output;
    }

    /**
     * This method finds the closest point on the path to the robot
     * 
     * @param path     the path the robot follows
     * @param currPose robot's current pose
     * @return the point on the path which is closest to the robot
     */
    TrajPoint get_closest_point(final Path path, final Pose currPose) {
        int ID = closestPoint;
        final ArrayList<TrajPoint> points = path.get_points(ID);
        TrajPoint output = points.get(0);
        double dist = currPose.dist(output);
        for (final TrajPoint p : points) {
            if (currPose.dist(p) < dist) {
                closestPoint = ID;
                dist = currPose.dist(p);
                output = p;
            }
            ID++;
        }
        return output;
    }

    /**
     * This method is used to calculate the lookup velocites at each trajectory
     * point
     * 
     * @param driveCharacterization characterization of the drivetrain
     * @param points                arraylist of points to set velocites for
     */
    void calc_velocity(final DriveCharacterization driveCharacterization, final ArrayList<TrajPoint> points) {
        for (int i = 1; i < points.size() - 1; i++) {
            if (points.get(i).curvature == 0)
                points.get(i).velocity = driveCharacterization.maxVelocity;
            else if (driveCharacterization.maxVelocity / (points.get(i).curvature * 50) < driveCharacterization.maxVelocity)
                points.get(i).velocity = driveCharacterization.maxVelocity / (points.get(i).curvature * 50);
            else
                points.get(i).velocity = driveCharacterization.maxVelocity;
        }
        points.get(0).velocity = 0;
        points.get(points.size() - 1).velocity = 0;
        double plausVel = 0;
        for (int i = 1; i < points.size() - 1; i++) {
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
        points.get(0).velocity = points.get(1).velocity;
    }
}