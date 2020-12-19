package org.firstinspires.ftc.teamcode.java.fieldmapping;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.java.utils.Hardware;


/**
 * The ActiveLocation uses odometry to find the real-time location of the robot.
 * This, along with a PathFinder, helps create a Field Mapping to allow us to
 * accurately move to specific positions in autonomous.
 */
public class ActiveLocation implements Runnable {

    // Hardware setup
    Hardware robot;
    private BNO055IMU imu;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;

    //########## VARIABLE SET UP ##########\\

    // X and Y based on encoder
    private volatile double internalCurrentY = 0;
    private volatile double internalCurrentX = 0;

    // Used for sensor values
    private double yEncoder = 0;
    private double xEncoder = 0;
    double angle = 0;

    // Field location
    double fieldXPosition;
    double fieldYPosition;

    // For stopping the thread
    private volatile boolean isRunning;

    // Static values for calculations
    final static double tickPerRotation = 8192;
    final static double wheelCircumference = 90 * Math.PI;

    public ActiveLocation(Hardware robot){
        this.robot = robot;
        frontLeftMotor = robot.frontLeftMotor;
        backRightMotor = robot.backRightMotor;
        imu = robot.imu;

        isRunning = true;
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
     */
    private static double distanceToTicks(double distance) {
        return ((distance / wheelCircumference) * tickPerRotation);
    }

    /**
     * Sets the start position
     * 
     * @param internalCurrentX The current X position mm
     * @param internalCurrentY The current Y position mm
     */
    public void setStartPosition(double internalCurrentX, double internalCurrentY) {
        this.internalCurrentY = internalCurrentY;
        this.internalCurrentX = internalCurrentX;
    }

    /**
     * Updates Relevant Values for ActiveLocation calculations
     *
     * This function updates both the X and Y values of the Encoder's position, as well as the angle
     * of the robot using the built in IMU on the Rev Hub.
     */
    private void updateSensors() {
        yEncoder = frontLeftMotor.getCurrentPosition();
        xEncoder = backRightMotor.getCurrentPosition();
        angle = imu.getAngularOrientation(/*AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS*/).firstAngle;
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
        //fieldXPosition = internalPreviousX;
        //fieldYPosition = internalCurrentY;
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
    public double getAngleInDegrees(){
        updateSensors();
        return Math.toDegrees(angle);
    }

    /**
     * Sets up the thread to stop
     */
    public void setStop() {
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
        //this.robot.init(hardwareMap);



        while (isRunning) {
            updateSensors();
            findFieldPosition();
        }
    }
}
