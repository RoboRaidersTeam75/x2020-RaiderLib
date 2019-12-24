package raiderlib.control;

/**
 * This class is used to store left and right drive commands
 */
public class DriveCommand {
    public double left_command;
    public double right_command;

    /**
     * Constructor for DriveCommand class
     * 
     * @param left_command  left command
     * @param right_command right command
     */
    public DriveCommand(final double left_command, final double right_command) {
        this.left_command = left_command;
        this.right_command = right_command;
    }
}