package org.firstinspires.ftc.teamcode.java.movement;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.java.util.MovementData;
import org.firstinspires.ftc.teamcode.java.util.PIDFController;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

import java.util.Locale;

/**
 * The AutoDriving class allows the robot to move to specified locations after calculating with
 * PathFinder.
 */
public class AutoDriving {

    private final PIDFController PIDFDrive;
    private final PIDFController PIDStrafe;
    private final PIDFController PIDFTurn;
    private final ActiveLocation activeLocation;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;
    private final Thread locationThread;
    private final PathFinder pathFinder;
    private final Thread pathThread;
    RobotHardware robot;
    //private AutoDriving autoDriving;


    public AutoDriving(PIDFController PIDFDrive, PIDFController PIDFStrafe, PIDFController PIDFTurn, RobotHardware robot) {
        this.PIDFDrive = PIDFDrive;
        this.PIDStrafe = PIDFStrafe;
        this.PIDFTurn = PIDFTurn;
        this.robot = robot;
        frontLeftMotor = robot.frontLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor = robot.backRightMotor;
        backLeftMotor = robot.backLeftMotor;

        activeLocation = new ActiveLocation(robot);
        locationThread = new Thread(activeLocation);
        locationThread.start();
        //AL.setStartPosition(0, 0, 270);

        pathFinder = new PathFinder(activeLocation);
        pathThread = new Thread(pathFinder);
        pathThread.start();
    }

    //TODO fill blank functions write aPID controller  add more functionality

    /**
     * drives to a point and stops using PID
     */
    public String stopAt(MovementData goal, double Vmax) {
        boolean arrived = false;
        //while (!arrived) {
        pathFinder.setDestination(goal);
        MovementData error = pathFinder.getEncoderPath();
        double[] speeds = calculateDrivePowers(Vmax, error);//PIDFDrive.calculateDrivePowers(Vmax, error);
        setMotorPowers(speeds);
        //arrived = true;
        /*
        if ((Math.abs(goal.getX() - AL.getFieldX()) < 10) && (Math.abs(goal.getY() - AL.getFieldY()) < 15) &&  (PF.getEncoderPath().getAngleInDegrees() <= 5)){
            return true;
        }

        return false;
*/
        //}
        return String.format(
                Locale.ENGLISH,
                "X: %.2f Y: %.2f A: %.2f",
                Math.abs(goal.getX() - activeLocation.getFieldX()),
                Math.abs(goal.getY() - activeLocation.getFieldY()),
                -pathFinder.getEncoderPath().getRawAngleInDegrees()
        );
        //return String.format(Locale.ENGLISH, "X: %.2f", (Math.abs(goal.getX() - AL.getFieldX())));
    }

    public double[] calculateDrivePowers(double maxV, MovementData errors) {
        return calculateDrivePowers(maxV, errors.getX(), errors.getY(), errors.getRawAngleInRadians());
    }

    public double[] calculateDrivePowers(double maxV, double xError, double yError, double angleError) {
        //aError = angleError;
        //angleError = ((Math.toDegrees(angleError) + 360) % 360);
        double strafe = PIDStrafe.calculateDrivePID(xError);
        double drive = PIDFDrive.calculateDrivePID(yError);
        double twist = PIDFTurn.calculateDrivePID(angleError);
        //double twist = 0;

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (Math.abs(speed) > max) {
                max = Math.abs(speed);
            }
        }
        if (max > maxV) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] *= maxV / max;
            }
        }

        return speeds;
    }

    /**
     * drives through a point without stopping
     */
    public  void passAt() {
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

    public String errorReport(MovementData goal) {
        return String.format(
                Locale.ENGLISH,
                "X: %.2f Y: %.2f A: %.2f",
                Math.abs(goal.getX() - activeLocation.getFieldX()),
                Math.abs(goal.getY() - activeLocation.getFieldY()),
                -pathFinder.getEncoderPath().getRawAngleInDegrees()
        );
        // return "X: "+Math.abs(goal.getX() - AL.getFieldX())+" Y: "+Math.abs(goal.getY() - AL.getFieldY())+" A: "+Math.abs(goal.getAngleInDegrees() - AL.getAngleInDegrees());
        // return "A: Math.abs("+goal.getAngleInDegrees()+"-"+AL.getAngleInDegrees()+") = "+(Math.abs(goal.getAngleInDegrees() - AL.getAngleInDegrees()));
        // double[] speeds = PIDF.calculateDrivePowers(Vmax, goal);
        // return String.format("1: %.2f 2: %.2f 3: %.2f 4: %.2f", speeds[0], speeds[1])

    }

    private void setMotorPowers(double[] speeds) {
        frontLeftMotor.setPower(speeds[0]);
        frontRightMotor.setPower(speeds[1]);
        backLeftMotor.setPower(speeds[2]);
        backRightMotor.setPower(speeds[3]);
    }
}
