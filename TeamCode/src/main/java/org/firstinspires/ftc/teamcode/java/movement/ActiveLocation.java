package org.firstinspires.ftc.teamcode.java.movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.java.util.MovementData;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;



/**
 * The ActiveLocation uses odometry to find the real-time location of the robot.
 * This, along with a PathFinder, helps create a Field Mapping to allow us to
 * accurately move to specific positions in autonomous.
 */
public class ActiveLocation implements Runnable {

    // Hardware setup
    private final BNO055IMU imu;
    private final DcMotor yDirectionEncoder;
    private final DcMotor xDirectionEncoder;

    //########## VARIABLE SET UP ##########\\

    // X and Y based on encoder
    private volatile double internalCurrentY = 0;
    private volatile double internalCurrentX = 0;

    // Used for sensor values
    private double yEncoder = 0;
    private double xEncoder = 0;
    double angle = 0;
    double resetAngle = 0;
    double startAngle = 0;

    // Field location
    double fieldXPosition = 0;
    double fieldYPosition = 0;

    // For stopping the thread
    private volatile boolean isRunning = true;

    // Static values for calculations
    final static double tickPerRotation = 8192;
    final static double wheelCircumference = 90 * Math.PI;

    double startX;

    /**
     * Creates an Active Location Tracker for the Robot given by a {@link RobotHardware}
     *
     * @param robot the HardwareMap set of the Robot
     */
    public ActiveLocation(RobotHardware robot){
        yDirectionEncoder = robot.frontLeftMotor;
        xDirectionEncoder = robot.backRightMotor;
        imu = robot.imu;
    }

    /**
     * Create an Active Location Tracker for the Robot
     *
     * @param xDirectionEncoder the encoder set up in the X Direction
     * @param yDirectionEncoder the encoder set up in the Y Direction
     * @param gyroscope the imu (aka gyroscope) to determine the angle of the robot
     */
    public ActiveLocation(DcMotor xDirectionEncoder, DcMotor yDirectionEncoder, BNO055IMU gyroscope) {
        this.yDirectionEncoder = yDirectionEncoder;
        this.xDirectionEncoder = xDirectionEncoder;
        this.imu = gyroscope;
    }

    /**
     * Converts Ticks to Distance
     *
     * To calculate the distance, this function calculates the fraction of a total rotation (which
     * can be greater than 1 rotation) and multiplies it by the circumference, getting the linear
     * distance from the rotational fraction.
     *
     * @param ticks tick value
     * @return mm distance value
     */
    private static double tickToDistance(double ticks) {
        return ((ticks / tickPerRotation) * wheelCircumference);
    }


    /**
     * Converts Distance to Ticks
     *
     * To calculate the ticks traveled, the function determines the fraction of a rotation (using
     * distance and the circumference) and multiplies by the constant ticksPerRotation, returning
     * the total ticks.
     *
     * @param distance mm value
     * @return returns tick value
     * @deprecated
     */
    @Deprecated
    private static double distanceToTicks(double distance) {
        return ((distance / wheelCircumference) * tickPerRotation);
    }

    /**
     * Sets the start position
     *
     * @param startX The current X position mm
     * @param startY The current Y position mm
     * @param startAngle       The starting angle in degrees
     */
    public void setStartPosition(double startX, double startY, double startAngle) {
        this.startAngle = Math.toRadians(startAngle);
        this.internalCurrentX = startX * Math.cos(this.startAngle) - startY * Math.sin(this.startAngle);
        this.internalCurrentY = startX * Math.sin(this.startAngle) + startY * Math.cos(this.startAngle);


    }

    /**
     * Sets the start position
     * @param location A movement data containing x and y position in mm and the angle in degrees
     */
    public void setStartPosition(MovementData location){
        this.setStartPosition(location.getX(), location.getY(), location.getAngleInDegrees());

    }

    /**
     * Updates Relevant Values for ActiveLocation calculations
     *
     * This function updates both the X and Y values of the Encoder's position, as well as the angle
     * of the robot using the built in IMU on the Rev Hub.
     */
    private void updateSensors() {
        yEncoder = yDirectionEncoder.getCurrentPosition();
        xEncoder = xDirectionEncoder.getCurrentPosition();
        angle = ((imu.getAngularOrientation(/*AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS*/).firstAngle) + startAngle - resetAngle);
        angle = ((angle + (2 * Math.PI)) % (2 * Math.PI));
    }

    /**
     * Updates the current position of the Robot on the Field
     *
     * This function calculates the change in the robot's positional values, recalculating the
     * position of the robot.
     */
    public void findFieldPosition() {
        double internalPreviousY = internalCurrentY;
        double internalPreviousX = internalCurrentX;
        synchronized (this) {
            internalCurrentY = tickToDistance(yEncoder);
            internalCurrentX = tickToDistance(xEncoder);
        }
        // Change in internal x and y values
        double deltaY = internalCurrentY - internalPreviousY;
        double deltaX = internalCurrentX - internalPreviousX;
        fieldXPosition += deltaX * Math.cos(angle) - deltaY * Math.sin(angle);
        fieldYPosition += deltaX * Math.sin(angle) + deltaY * Math.cos(angle);
    }

    /**
     * Gets the Robot's Position on the field on the Y-Axis
     * 
     * @return Returns the robot's Y position on the field (mm)
     */
    public double getFieldY() {
        updateSensors();
        findFieldPosition();
        return fieldYPosition;
    }

    /**
     * Gets the Robot's Position on the field on the X-Axis
     * 
     * @return returns X position on the field (mm)
     */
    public double getFieldX() {
        updateSensors();
        findFieldPosition();
        //return startX;
        return fieldXPosition;
    }

    /**
     * Gets the Robot's Angle on the Field, relative to its initial angle
     * 
     * @return returns angle
     */
    public double getAngle() {
        updateSensors();
        return angle;
    }

    /**
     * Get's the Robot's Angle on the field, in degrees
     * @return returns angle in degrees
     */
    public double getAngleInDegrees(){
        updateSensors();
        return ((Math.toDegrees(angle) + 360) % 360);
    }

    /**
     * Takes the current angle and sets that value to 0 in the robots point of view
     */
    public void resetAngle(){
        updateSensors();
        resetAngle = angle;
    }

    /**
     * Sets up the thread to stop
     */
    public void stop() {
        isRunning = false;
    }


    /**
     * Sets up and starts the thread running in the background
     *
     * This function determines what the class will do when set to run on an individual thread,
     * maintaining the current position of the robot so that autonomous can update and properly
     * calculate where the robot needs to go and when the robot reached said position.
     */
    @Override
    public void run() {
        while (isRunning) {
            updateSensors();
            findFieldPosition();
        }
    }
}
