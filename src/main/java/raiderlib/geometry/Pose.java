package raiderlib.geometry;

import raiderlib.control.DriveCharacterization;

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
     * @param p pose object added
     * @return sum of poses
     */
    public Pose add(final Pose p) {
        this.x += p.x;
        this.y += p.y;
        this.theta += p.theta;
        return this;
    }
    /**
     * This method updates a non-linear estimation of the robot's position
     * @param driveCharacterization drive characterization
     * @param deltaLeft distance left side has traveled
     * @param deltaRight distance right side has traveld
     * @return instance of the new pose
     */
    public Pose update(DriveCharacterization driveCharacterization, double deltaLeft, double deltaRight){
        double deltaDistance = (deltaLeft + deltaRight)/2;
        double deltaTheta = (deltaRight - deltaLeft)/driveCharacterization.trackWidth;
        double dx = deltaDistance * Math.cos(this.theta + deltaTheta/2);
        double dy = deltaDistance * Math.sin(this.theta + deltaTheta/2);
        this.x += dx;
        this.y += dy;
        this.theta += deltaTheta;
        return new Pose(this.x, this.y, this.theta);
    }
    /**
     * This method updates a non-linear estimation of the robot's position
     * @param gyro gyro reading(in radians)
     * @param deltaLeft distance left side has traveled
     * @param deltaRight distance right side has traveled
     * @return instance of new Pose
     */
    public Pose update(double gyro, double deltaLeft, double deltaRight){
        double deltaDistance = (deltaLeft + deltaRight)/2;
        double deltaTheta = gyro - this.theta;
        double dx = deltaDistance * Math.cos(this.theta + deltaTheta/2);
        double dy = deltaDistance * Math.sin(this.theta + deltaTheta/2);
        this.x += dx;
        this.y += dy;
        this.theta += deltaTheta;
        return new Pose(this.x, this.y, this.theta);
    }

}