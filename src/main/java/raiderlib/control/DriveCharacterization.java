package raiderlib.control;

public class DriveCharacterization {

    public final double maxVelocity;
    public final double maxAcceleration;
    public final double trackWidth;

    /**
     * Constructor for Drivecharacterization class
     * 
     * @param maxVelocity     max veloicty
     * @param maxAcceleration max acceleration
     * @param trackWidth      trackWidth of robot
     */
    public DriveCharacterization(double maxVelocity, double maxAcceleration, double trackWidth) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.trackWidth = trackWidth;
    }

}