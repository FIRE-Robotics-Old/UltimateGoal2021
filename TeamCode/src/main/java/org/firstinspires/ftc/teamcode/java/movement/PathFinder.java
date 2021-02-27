package org.firstinspires.ftc.teamcode.java.movement;

import org.firstinspires.ftc.teamcode.java.util.*;

import static org.firstinspires.ftc.teamcode.java.util.Constants.PI;
import static org.firstinspires.ftc.teamcode.java.util.Constants.TAU;


/**
 * The PathFinder finds the correct path the Robot Needs to Take to move to a different point.
 */
public class PathFinder implements Runnable {

    //TODO: Implement angle calculations in PathFinder

    /**
     * The ActiveLocation Object to Use for Reference Calculations
     */
    private final ActiveLocation activeLocation;
    /**
     * The Destination the PathFinder is calculating for
     */
    private MovementData destination;

    /**
     * The Movement Necessary in the X Direction
     */
    private double xToMove = 0;
    /**
     * The Movement Necessary in the Y Direction
     */
    private double yToMove = 0;
    /**
     * The Movement Necessary in the θ axis
     *
     * TODO: Change to {@link Angle}?
     */
    private double aToMove = 0;

    /**
     * A Boolean to Determine if PathFinder is Still Running
     */
    private volatile boolean isRunning = true;

    /**
     * Construct a PathFinder without a Destination
     *
     * @param activeLocation the Active Location of the Robot
     */
    public PathFinder(ActiveLocation activeLocation) {
        this.activeLocation = activeLocation;
    }

    /**
     * Construct a PathFinder object using the destination and the current location
     *
     * @param activeLocation the Active Location of the Robot
     * @param destination the Destination the Robot needs to travel to
     */
    public PathFinder(ActiveLocation activeLocation, MovementData destination) {
        this.activeLocation = activeLocation;
        this.destination = destination;
    }

    /**
     * Constructs PathFinder without an Angle
     *
     * @param activeLocation the Active Location of the Robot
     * @param x the destination x coordinate
     * @param y the destination y coordinate
     */
    public PathFinder(ActiveLocation activeLocation, double x, double y) {
        // TODO: Should the default angle be zero or the current angle?
        this(activeLocation, new MovementData(x, y, Angle.fromDegrees(0)));
    }

    /**
     * Constructs a PathFinder without Abstraction of MovementData
     *
     * @param activeLocation the Active Location of the Robot
     * @param x the destination x coordinate
     * @param y the destination y coordinate
     * @param angle the destination angle
     */
    public PathFinder(ActiveLocation activeLocation, double x, double y, Angle angle) {
        this(activeLocation, new MovementData(x, y, angle));
    }

    /**
     * Returns the current destination that the Robot is attempting to travel to
     *
     * @return the destination the robot is travelling to
     */
    public MovementData getDestination() {
        updateEncoderPath();
        return destination;
    }

    /**
     * Updates the destination that the Robot is travelling to
     *
     * @param destination the new destination
     */
    public void setDestination(MovementData destination) {
        updateEncoderPath();
        this.destination = destination;
    }

    // TODO: Remove reliance on deprecated methods
    /**
     * Updates the destination using only the x and y coordinates
     *
     * @param x the destination x coordinate
     * @param y the destination y coordinate
     */
    public void setDestination(double x, double y) {
        this.setDestination(MovementData.withDegrees(x, y, 0));
    }

    // TODO: Add more consistent naming (rotation → Angle or alpha)
    /**
     * Updates the destination
     *
     * @param x the destination x coordinate
     * @param y the destination y coordinate
     * @param rotation the destination angle
     */
    public void setDestination(double x, double y, double rotation) {
        this.setDestination(MovementData.withDegrees(x, y, rotation));
    }

    /**
     * Updates the destination
     *
     * @param translation the destination vector
     * @param rotation the destination angle
     */
    public void setDestination(Vector2d translation, Angle rotation) {
        this.setDestination(new MovementData(translation, rotation));
    }

    /**
     * Calculates the positions that the robot encoders need to move to using the current position,
     * allowing instantaneous calculation of the movement the robot needs to make.
     */
    public void updateEncoderPath() {
        synchronized (this) {
            if (activeLocation == null || destination == null) return;

            final double deltaX = destination.getX() - activeLocation.getFieldX();
            final double deltaY = destination.getY() - activeLocation.getFieldY();

            xToMove = deltaX * Math.cos(activeLocation.angle) + deltaY * Math.sin(activeLocation.angle);
            yToMove = deltaY * Math.cos(activeLocation.angle) - deltaX * Math.sin(activeLocation.angle);
            calculateTurn();
        }
    }


    /**
     * Calculates the angle change that the robot gyro needs to do
     */
    public void calculateTurn() {
        //subtract angles to figure out direction?
        synchronized (this) {
            if (activeLocation == null || destination == null) return;
            aToMove = (destination.getAngleInRadians() - activeLocation.getAngle());
            if (aToMove > PI) {
                aToMove = -(TAU - aToMove);
            } else if (aToMove < -PI) {
                aToMove = -(TAU - Math.abs(aToMove));
            }
        }
    }

    /**
     * @return a {@link Coordinate} which contains the change values for Robot Encoders
     */
    public MovementData getEncoderPath() {
        updateEncoderPath();
        //calculateTurn();
        return MovementData.withRadians(xToMove, yToMove, aToMove);
    }


    /**
     * Stops the Thread
     */
    public void stop() {
        this.isRunning = false;
    }

    /**
     * Resumes the Thread
     *
     * TODO: Check if it works
     */
    public void resume() {
        this.isRunning = true;
    }

    /**
     * Constantly Updates Encoder Path in PathFinder
     *
     * TODO: Check if keeping this in a thread is really necessary
     */
    @Override
    public void run() {
        while (isRunning) {
            updateEncoderPath();
        }
    }
}
