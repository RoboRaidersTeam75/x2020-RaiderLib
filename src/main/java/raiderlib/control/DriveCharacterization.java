package raiderlib.control;

public class DriveCharacterization {

    public final double maxVelocity;
    public final double maxAcceleration;
    public final double trackWidth;
    public final double loopTime;
    /**
     * Constructor for Drivecharacterization class
     * @param maxVelocity max veloicty
     * @param maxAcceleration max acceleration
     * @param trackWidth trackWidth of robot
     * @param loopTime how often your drive is updated
     */
    public DriveCharacterization(double maxVelocity, double maxAcceleration, double trackWidth, double loopTime){
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.trackWidth = trackWidth;
        this.loopTime = loopTime;
    }


}