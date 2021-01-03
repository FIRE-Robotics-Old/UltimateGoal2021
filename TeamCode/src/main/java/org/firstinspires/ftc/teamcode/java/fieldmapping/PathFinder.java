package org.firstinspires.ftc.teamcode.java.fieldmapping;

import org.firstinspires.ftc.teamcode.java.util.Coordinate;
import org.firstinspires.ftc.teamcode.java.util.MovementData;

/**
 * The PathFinder finds the correct path the Robot Needs to Take to move to a different point.
 */
public class PathFinder implements Runnable {

    //TODO: Implement angle calculations in PathFinder

    private final ActiveLocation activeLocation;
    private MovementData destination;

    private double xToMove;
    private double yToMove;
    private double aToMove;

    private volatile boolean isRunning = true;


    public PathFinder(ActiveLocation activeLocation, MovementData destination) {
        this.activeLocation = activeLocation;
        this.destination = destination;
    }

    public PathFinder(ActiveLocation activeLocation, double x, double y) {
        this(activeLocation, MovementData.withDegrees(x, y, 0));
    }

    public PathFinder(ActiveLocation activeLocation) {
        this.activeLocation = activeLocation;
    }

    public PathFinder(ActiveLocation activeLocation, double x, double y, double alpha) {
        this(activeLocation, MovementData.withDegrees(x, y, alpha));
    }

    public MovementData getDestination() {
        updateEncoderPath();
        return destination;
    }

    public void setDestination(MovementData destination) {
        updateEncoderPath();
        this.destination = destination;
    }

    public void setDestination(double x, double y) {
        this.setDestination(MovementData.withDegrees(x, y, 0));
    }

    public void setDestination(double x, double y, double alpha) {
        this.setDestination(MovementData.withDegrees(x, y, alpha));
    }


    public void setDestination(Coordinate coordinate, double alpha) {
        this.setDestination(MovementData.withDegrees(coordinate, alpha));
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
     * calculates the angle change that the robot gyro needs to do
     */
    public void calculateTurn() {
        //subtract angles to figure out direction?
        synchronized (this) {
            if (activeLocation == null || destination == null) return;
            aToMove = (destination.getAngleInRadians() - activeLocation.getAngle());
            if (aToMove > Math.PI) {
                aToMove = -1 * ((Math.PI * 2) - aToMove);
            } else if (aToMove < -Math.PI) {
                aToMove = (Math.PI * 2) - Math.abs(aToMove);
            }
        }
    }

    /**
     * @return a @{link Coordinate} which contains the change values for Robot Encoders
     */
    public MovementData getEncoderPath() {
        updateEncoderPath();
        //calculateTurn();
        return MovementData.withRadians(xToMove, yToMove, aToMove);
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
