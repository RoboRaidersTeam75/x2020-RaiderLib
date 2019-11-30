package raiderlib.control;

import java.util.ArrayList;

import raiderlib.geometry.Point;
import raiderlib.geometry.Pose;
import raiderlib.path.Path;
import raiderlib.path.TrajPoint;
/**
 * This class is used as a PurePursuitController.
 * To pursuit a path, use the pursuit_path method
 * To pursuit a point, use the pursuit_point method
 */
public class PurePursuitController {
    DriveCharacterization driveCharacterization;
    double lookAheadDistance;
    /**
     * Constructor for PurePursuitController class
     * @param driveCharacterization driveCharacterization of robot
     * @param lookAheadDistance max lookahead distance
     */
    public PurePursuitController(DriveCharacterization driveCharacterization, double lookAheadDistance) {
        this.driveCharacterization = driveCharacterization;
        this.lookAheadDistance = lookAheadDistance;
    }
    /**
     * This method is used to pursuit a path
     * @param path path to be pursuited
     * @param currPose robot's current pose
     * @return a DriveCommand for the robot to follow
     */
    public DriveCommand pursuit_path(Path path, Pose currPose) {
        TrajPoint closestPoint = get_closest_point(path, currPose);
        Point lookAheadPoint = get_lookAhead_point(path, currPose,
                (1 / closestPoint.curvature < this.lookAheadDistance) ? 1 / closestPoint.curvature
                        : this.lookAheadDistance);
        return pursuit_point(lookAheadPoint, currPose, closestPoint.velocity);
    }
    /**
     * This method is used to pursuit a point
     * @param p point to pursuit
     * @param currPose robot's current pose
     * @param velocity velocity to pursuit at
     * @return a DriveCommand for the robot to follow
     */
    public DriveCommand pursuit_point(Point p, Pose currPose, double velocity) {
        double arcCurve = get_arc_curvature(p, currPose);
        return new DriveCommand(velocity * (2 + arcCurve * this.driveCharacterization.trackWidth),
                velocity * (2 - arcCurve * this.driveCharacterization.trackWidth));
    }
    /**
     * This method is used to calculate the curvature of the arc to a lookahead point
     * @param lookAheadPoint the lookahead point
     * @param currPose robot's current pose
     * @return the curvature of the arc
     */
    double get_arc_curvature(Point lookAheadPoint, Pose currPose) {
        double a = -Math.tan(currPose.theta);
        double c = Math.tan(currPose.theta) * currPose.x - currPose.y;
        double x = Math.abs(a * lookAheadPoint.x + lookAheadPoint.y + c) / Math.sqrt(a * a + 1);
        int sign = ((Math.sin(currPose.theta) * (lookAheadPoint.x - currPose.x)
                - Math.cos(currPose.theta) * (lookAheadPoint.y - currPose.y)) > 0) ? 1 : -1;
        return sign * 2 * x / (Math.pow(currPose.dist(lookAheadPoint), 2));
    }
    /**
     * This method is used to find the lookahead point on a path
     * @param path the path the robot follows
     * @param currPose robot's current pose
     * @param lookAheadDistance lookahead distance of robot
     * @return the point on the path within the lookahead radius. If there is more than one, the robot chooses the later one on the path
     */
    TrajPoint get_lookAhead_point(Path path, Pose currPose, double lookAheadDistance) {
        int ID = path.getLookAheadID();
        ArrayList<TrajPoint> points = path.get_points(ID);
        TrajPoint output = points.get(0);
        for (TrajPoint p : points) {
            if ((Math.pow(p.x - currPose.x, 2) + Math.pow(p.y - currPose.y, 2)) <= Math.pow(lookAheadDistance, 2) + 50
                    && (Math.pow(p.x - currPose.x, 2) + Math.pow(p.y - currPose.y, 2)) >= Math.pow(lookAheadDistance, 2)
                            - 50) {
                path.setLookAheadID(ID);
                output = p;
            }
            ID++;
        }
        return output;
    }
    /**
     * This method finds the closest point on the path to teh robot
     * @param path the path the robot follows
     * @param currPose robot's current pose
     * @return the point on the path which is closest to the robot
     */
    TrajPoint get_closest_point(Path path, Pose currPose) {
        int ID = path.getClosestPointID();
        ArrayList<TrajPoint> points = path.get_points(ID);
        TrajPoint output = points.get(0);
        double dist = currPose.dist(output);
        for (TrajPoint p : points) {
            if (currPose.dist(p) < dist) {
                path.setClosestPointID(ID);
                dist = currPose.dist(p);
                output = p;
            }
            ID++;
        }
        return output;
    }
}