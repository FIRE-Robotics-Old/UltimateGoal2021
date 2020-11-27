package org.firstinspires.ftc.teamcode.fieldmapping;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

/**
 * The ActiveLocation uses odometry to find the real-time location of the robot. This,
 * along with a PathFinder, helps create a Field Mapping to allow us to accurately move to specific
 * positions in autonomous.
 */
public class ActiveLocation implements Runnable{ // add implements Runnable{

    //Hardware setup
    Hardware robot = new Hardware();
    private BNO055IMU imu;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;

    //var setup

    //change in internal x and y values
    private volatile double deltaX = 0;
    private volatile double deltaY = 0;
    //X and Y based on encoder
    private volatile double internalCurrentY = 0;
    private volatile double internalCurrentX = 0;
    private volatile double internalPreviousY = 0;
    private volatile double internalPreviousX = 0;
    //used for sensor values
    private double yEncoder = 0;
    private double xEncoder = 0;
    double angle = 0;
    //Field location
    double fieldXPositon;
    double fieldYPostion;
    //For stopping the thread
    private volatile boolean stop = false;

    //Static values for calculations
    final static double tickPerRotation = 8192;
    final static double wheelCircumference = 90 * Math.PI;

    //TODO Write JavaDoc Comment
    /**
     * Figures out distance based on current encoder position over total ticks per rotation and muliplies it by circumfernce
     * @param ticks tick value
     * @return mm distance value
     */
    private static double tickToDistance(double ticks){
        double currentTicks = ticks;
        return ((currentTicks/tickPerRotation) * wheelCircumference);
    }

    /**
     * Figures out ticks based on distance over wheel circumference  multiplies it by ticksPerRotation
     * @param distance mm value
     * @return returns tick value
     */
    private static double DistanceToTicks(double distance){

        return ((distance/wheelCircumference)*tickPerRotation);
    }

    /**
     * Sets the start postion
     * @param internalCurrentX The current X position mm
     * @param internalCurrentY The current Y position mm
     */

    public void setStartPosition(double internalCurrentX, double internalCurrentY){
        this.internalCurrentY = internalCurrentY;
        this.internalCurrentX = internalCurrentX;
    }

    /**
     * Updates the yEncoder, X encoder, and imu values
     */
    private void updateSensors(){
        yEncoder = frontLeftMotor.getCurrentPosition();
        xEncoder = backRightMotor.getCurrentPosition();
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
    }

    /**
     * Calculates where on the field the robot is located
     */
    public void findFieldPosition(){
        internalPreviousY = internalCurrentY;
        internalPreviousX = internalCurrentX;
        synchronized (this) {
            internalCurrentY = tickToDistance(yEncoder);
            internalCurrentX = tickToDistance(xEncoder);
        }
        deltaY = internalCurrentY - internalPreviousY;
        deltaX = internalCurrentX - internalPreviousX;
        fieldXPositon += deltaX * Math.cos(Math.toRadians(angle)) - deltaY * Math.sin(Math.toRadians(angle));
        fieldYPostion += deltaX * Math.sin(Math.toRadians(angle)) + deltaY * Math.cos(Math.toRadians(angle));
    }

    /**
     * Gets Field Y Position
     * @return Returns the robot's Y position on the field (mm)
     */
    public double getFieldYPostion(){
        updateSensors();
        findFieldPosition();
        return fieldYPostion;
    }

    /**
     * Gets Field X Position
     * @return returns X postion on the field (mm)
     */
    public double getFieldXPositon(){
        updateSensors();
        findFieldPosition();
        return fieldXPositon;
    }

    /**
     * Gets robot angle
     * @return returns angle
     */
    public double getAngle(){
        return angle;
    }

    /**
     * Stops the thread
     */
    public void setStop(){
        stop = true;
    }

    /**
     * The function that the thread needs
     */
    @Override
    public void run() {

        robot.init(hardwareMap);

        frontLeftMotor = robot.frontLeftMotor;
        backRightMotor = robot.backRightMotor;


        imu = robot.imu;
        while (!stop){
            updateSensors();
            findFieldPosition();
        }
    }

}
