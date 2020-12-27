package org.firstinspires.ftc.teamcode.java.fieldmapping;

import org.firstinspires.ftc.teamcode.java.utils.Coordinate;

/**
 * The PathFinder finds the correct path the Robot Needs to Take to move to a different point.
 */
public class PathFinder implements Runnable {

    //TODO: Implement angle calculations in PathFinder

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
    public PathFinder(ActiveLocation activeLocation){
        this.activeLocation = activeLocation;
    }

    public Coordinate getDestination() {
        updateEncoderPath();
        return destination;
    }
    public void setDestination(Coordinate destination) {
        updateEncoderPath();
        this.destination = destination;
    }
    public void setDestination(double x, double y){
        this.setDestination(new Coordinate(x,y));
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
        }
    }


    /**
     *calculates the angle change that the robot gyro needs to do
     */ //bad name
    public void updateGyroPath(){

    }

    /**
     * @return a @{link Coordinate} which contains the change values for Robot Encoders
     */
    public Coordinate getEncoderPath() {
        updateEncoderPath();
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
