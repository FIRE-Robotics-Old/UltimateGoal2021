package org.firstinspires.ftc.teamcode.fieldmapping;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

/**
 * The ActiveLocation uses odometry to find the real-time location of the robot. This,
 * along with a PathFinder, helps create a Field Mapping to allow us to accurately move to specific
 * positions in autonomous.
 */
public class ActiveLocation implements Runnable{ // add implements Runnable{


    Hardware robot = new Hardware();
    private BNO055IMU imu;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private volatile double deltaX;
    private volatile double deltaY;
    private volatile double current_Y;
    private volatile double current_X;
    private volatile double previous_Y;
    private volatile double previous_X;


    final static double tickPerRotation = 8192;
    final static double wheelCircumference = 90 * Math.PI;

    public ActiveLocation(DcMotor frontLeftMotor, DcMotor backRightMotor){
        this.frontLeftMotor = frontLeftMotor;
        this.backRightMotor = backRightMotor;

    }

    //TODO Write JavaDoc Comment
    /**
     * Figures out distance based on current encoder position over total ticks per rotation and muliplies it by circumfernce
     * @param ticks
     * @param encoder
     * @return
     */
    public static double tickToDistance(double ticks, DcMotor encoder){
        double currentTicks = encoder.getCurrentPosition();
        return ((currentTicks/tickPerRotation) * wheelCircumference);
    }

    public static double DistanceToTicks(double distance, DcMotor encoder){

        return ((distance/wheelCircumference)*tickPerRotation);
    }

    @Override
    public void run() {

        robot.init(hardwareMap);

        frontLeftMotor = robot.frontLeftMotor;
        backRightMotor = robot.backRightMotor;

        imu = robot.imu;

    }

}
