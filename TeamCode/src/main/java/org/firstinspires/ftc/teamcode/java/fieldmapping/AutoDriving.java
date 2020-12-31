package org.firstinspires.ftc.teamcode.java.fieldmapping;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.java.utils.MovementData;
import org.firstinspires.ftc.teamcode.java.utils.PIDFController;
import org.firstinspires.ftc.teamcode.java.utils.RobotHardware;

/**
 * The AutoDriving class allows the robot to move to specified locations after calculating with
 * PathFinder.
 */
public class AutoDriving {

    private final PIDFController PIDF;
    private final ActiveLocation AL;
    private final DcMotor frontRightMotor;
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;
    private final Thread locationThread;
    private final PathFinder PF;
    private final Thread pathThread;
    RobotHardware robot;
    private AutoDriving autoDriving;


    public AutoDriving(PIDFController PIDF, RobotHardware robot) {
        this.PIDF = PIDF;
        this.robot = robot;
        frontLeftMotor = robot.frontLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor = robot.backRightMotor;
        backLeftMotor = robot.backLeftMotor;

        AL = new ActiveLocation(robot);
        locationThread = new Thread(AL);
        locationThread.start();

        PF = new PathFinder(AL);
        pathThread = new Thread(PF);
        pathThread.start();

    }
    //TODO fill blank functions write aPID controller  add more functionality

    /**
     * drives to a point and stops using PID
     */
    public boolean stopAt(MovementData goal, double Vmax) {
        boolean arrived = false;
        while (!arrived) {
            PF.setDestination(goal);
            MovementData error = PF.getEncoderPath();
            double[] speeds = PIDF.calculateDrivePowers(Vmax, error);
            setMotorPowers(speeds);
            if ((Math.abs(goal.getX() - AL.getFieldX()) < 50) && (Math.abs(goal.getY() - AL.getFieldY()) < 50) && (Math.abs(goal.getAngleInDegrees() - AL.getAngleInDegrees()) <= 15)) {
                arrived = true;
            }
        }
        return true;
    }

    /**
     * drives through a point without stopping
     */
    public  void passAt(){
    }

    /**
     * rotates to an angle and keeps it using PID
     */
    public void rotateTo() {
    }

    /**
     * uses all the above functions for combination of driving and tuning
     */
    public void drive() {

    }

    private void setMotorPowers(double[] speeds) {
        frontLeftMotor.setPower(speeds[0]);
        frontRightMotor.setPower(speeds[1]);
        backLeftMotor.setPower(speeds[2]);
        backRightMotor.setPower(speeds[3]);
    }
}
