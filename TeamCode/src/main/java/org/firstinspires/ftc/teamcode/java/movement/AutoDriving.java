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
    public boolean stopAt(MovementData goal, double Vmax) {
        boolean arrived = false;
        while (!arrived) {
            pathFinder.setDestination(goal);
            MovementData error = pathFinder.getEncoderPath();
            double[] speeds = calculateDrivePowers(Vmax, error);//PIDFDrive.calculateDrivePowers(Vmax, error);
            setMotorPowers(speeds);
            if ((Math.abs(goal.getX() - activeLocation.getFieldX()) < 40) && (Math.abs(goal.getY() - activeLocation.getFieldY()) < 40) && (Math.abs(pathFinder.getEncoderPath().getAngleInDegrees()) <= 25)) {
                arrived = true;
            }
        }

        turnOff();
        //arrived = true;
        return arrived;

        //}
//        return String.format(
//                Locale.ENGLISH,
//                "X: %.2f Y: %.2f A: %.2f",
//                Math.abs(goal.getX() - activeLocation.getFieldX()),
//                Math.abs(goal.getY() - activeLocation.getFieldY()),
//                -pathFinder.getEncoderPath().getRawAngleInDegrees()
//        );
        //return String.format(Locale.ENGLISH, "X: %.2f", (Math.abs(goal.getX() - AL.getFieldX())));
    }

    public double[] calculateDrivePowers(double maxV, MovementData errors) {
        return calculateDrivePowers(maxV, errors.getX(), errors.getY(), errors.getRawAngleInRadians());
    }

    public double[] calculateDrivePowers(double maxV, double xError, double yError, double angleError) {
        //aError = angleError;
        //angleError = ((Math.toDegrees(angleError) + 360) % 360);
        double strafe = PIDStrafe.calculatePID(xError);
        double drive = PIDFDrive.calculatePID(yError);
        double twist = PIDFTurn.calculatePID(angleError);
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

    public void turnOff(){
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    /**
     * Sets start location in mm and degrees
     * @param location location you are starting at a MovementData
     */
    public void setStartLocation(MovementData location){
        activeLocation.setStartPosition(location);

    }
    public void setStartLocation(double x, double y, double angle){
        this.setStartLocation(MovementData.withDegrees(x,y,angle));
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
    public String errorReport(double x,double y,double z){
        return this.errorReport(MovementData.withDegrees(x,y,z));
    }


    private void setMotorPowers(double[] speeds) {
        frontLeftMotor.setPower(speeds[0]);
        frontRightMotor.setPower(speeds[1]);
        backLeftMotor.setPower(speeds[2]);
        backRightMotor.setPower(speeds[3]);
    }
}
