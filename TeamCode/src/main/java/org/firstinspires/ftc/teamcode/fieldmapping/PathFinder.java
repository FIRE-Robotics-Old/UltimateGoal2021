package org.firstinspires.ftc.teamcode.fieldmapping;

import org.firstinspires.ftc.teamcode.Coordinate;


/**
 * The PathFinder finds the correct path the Robot Needs to Take to move to a different point.
 */
public class PathFinder implements Runnable {

    // TODO: Needs to use a thread
    // TODO: Need to use ActiveLocation
    // TODO: Set Stop

    private final ActiveLocation activeLocation;
    private Coordinate destination;

    private double xToMove;
    private double yToMove;

    private volatile boolean isRunning = true;


    public PathFinder(ActiveLocation activeLocation, Coordinate destination) {
        this.activeLocation = activeLocation;
        this.destination = destination;
    }

    public PathFinder(ActiveLocation activeLocation, double x, double y) {
        this(activeLocation, new Coordinate(x, y));
    }

    public Coordinate getDestination() {
        return destination;
    }

    public void setDestination(Coordinate destination) {
        updateEncoderPath();
        this.destination = destination;
    }

    /**
     * Calculates the positions that the robot encoders need to move to using the current position,
     * allowing instantaneous calculation of
     *
     */
    public void updateEncoderPath() {
        synchronized (this) {
            if (activeLocation == null || destination == null) return;

            final double deltaX = destination.getX() - activeLocation.getFieldX();
            final double deltaY = destination.getY() - activeLocation.getFieldY();

            xToMove = deltaX * Math.cos(activeLocation.angle) + deltaY * Math.sin(activeLocation.angle);
            yToMove = deltaY * Math.cos(activeLocation.angle) - deltaX * Math.sin(activeLocation.angle);
        }
    }

    /**
     *
     * @return a @{link Coordinate} which contains the change values for Robot Encoders
     */
    public Coordinate getEncoderPath() {
        return new Coordinate(xToMove, yToMove);
    }

    public void stop() {
        this.isRunning = false;
    }

    public void resume() {
        this.isRunning = true;
    }

    @Override
    public void run() {
        while (isRunning) {
            updateEncoderPath();
        }
    }
}
