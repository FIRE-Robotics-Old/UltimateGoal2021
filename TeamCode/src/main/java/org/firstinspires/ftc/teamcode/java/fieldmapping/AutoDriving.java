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
    private final DcMotor frontRightMotor;
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;
    RobotHardware robot = new RobotHardware();

    public AutoDriving(PIDFController PIDF, RobotHardware robot) {
        this.PIDF = PIDF;
        //this.robot.init(robot);
        frontLeftMotor = robot.frontLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor = robot.backRightMotor;
        backLeftMotor = robot.backLeftMotor;

    }
    //TODO fill blank functions write aPID controller  add more functionality

    /**
     * drives to a point and stops using PID
     */
    public void stopAt(MovementData goal, double Vmax) {
        double[] speeds = PIDF.calculateDrivePowers(Vmax, goal);
        setMotorPowers(speeds);

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
