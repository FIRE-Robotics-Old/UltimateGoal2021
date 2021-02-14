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
	private double defaultVmax = 0;
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

        pathFinder = new PathFinder(activeLocation);
        pathThread = new Thread(pathFinder);
        pathThread.start();
    }
    public void setDefaultVmax(double defaultVmax){
        this.defaultVmax = defaultVmax;
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
            if ((Math.abs(goal.getX() - activeLocation.getFieldX()) < 20) && (Math.abs(goal.getY() - activeLocation.getFieldY()) < 20) && (Math.abs(pathFinder.getEncoderPath().getAngleInDegrees()) <= 15)) {
                arrived = true;
            }
        }
        turnOff();
        return arrived;
    }
    public boolean stopAt(MovementData goal){
        return stopAt(goal, defaultVmax);
    }

    public double[] calculateDrivePowers(double maxV, MovementData errors) {
        return calculateDrivePowers(maxV, errors.getX(), errors.getY(), errors.getRawAngleInRadians());
    }

    public double[] calculateDrivePowers(double maxV, double xError, double yError, double angleError) {
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
    public boolean rotateTo(double angle, double Vmax) {
        MovementData goal = MovementData.withDegrees(activeLocation.getFieldX(),activeLocation.getFieldY(),angle);
        return stopAt(goal,Vmax);
    }
    public boolean rotateTo(double angle){
        return rotateTo(angle, defaultVmax);
    }
    /**
     * uses all the above functions for combination of driving and tuning
     */
    public boolean driveTo(double x, double y, double Vmax) {
        MovementData goal = MovementData.withDegrees(x,y,activeLocation.getAngleInDegrees());
        return stopAt(goal,Vmax);
    }
    public boolean driveXY(double x, double y, double Vmax){
        MovementData goal = MovementData.withDegrees((activeLocation.getFieldX()+x),(activeLocation.getFieldY()+y), activeLocation.getAngleInDegrees());
        boolean arrived = stopAt(goal,Vmax);
        return arrived;
    }
    
    public boolean driveTo(double x, double y){
        return driveTo(x,y, defaultVmax);
    }

    public boolean driveX(double x, double Vmax) {
        MovementData goal = MovementData.withDegrees(x,activeLocation.getFieldY(),activeLocation.getAngleInDegrees());
        return stopAt(goal,Vmax);
    }
    public boolean driveX(double x){
        return driveX(x, defaultVmax);
    }
    public boolean driveY(double y, double Vmax) {
        MovementData goal = MovementData.withDegrees(activeLocation.getFieldX(),y,activeLocation.getAngleInDegrees());
        return stopAt(goal,Vmax);
    }
    public boolean driveY(double y){
        return driveY(y, defaultVmax);
    }

    public boolean freeRotate(double angle, double Vmax) {
        boolean arrived = false;
        MovementData goal = MovementData.withDegrees(0, 0, angle);
        freeMove(goal,Vmax, 2);
        turnOff();
        return arrived;
    }
    public boolean freeRotate(double angle){
        return freeRotate(angle, defaultVmax);
    }

    public boolean freeDriveX(double x, double Vmax){
        boolean arrived = false;
        MovementData goal = MovementData.withDegrees(x, 0, 0);
        freeMove(goal,Vmax, 0);
        turnOff();
        return arrived;
    }
    public boolean freeDriveX(double x){
        return freeDriveX(x, defaultVmax);
    }
    public boolean freeDriveY(double y, double Vmax){
        boolean arrived = false;
        MovementData goal = MovementData.withDegrees(y, 0, 0);
        freeMove(goal,Vmax, 0);
        turnOff();
        return arrived;
    }
    public boolean freeDriveY(double y){
        return freeDriveY(y, defaultVmax);
    }
    public boolean freeDriveXY(double x, double y, double Vmax){
        boolean arrived = false;
        MovementData goal = MovementData.withDegrees(x, y, 0);
        freeMove(goal,Vmax,7); //Use large number to get default
        turnOff();
        return arrived;
    }
    public boolean freeDriveXY(double x, double y){
        return freeDriveXY(x,y, defaultVmax);
    }

    private boolean freeMove(MovementData goal, double Vmax, int which){ //Keep private/make others like this private
        boolean arrived = false;
        while (!arrived) {
            pathFinder.setDestination(goal);
            MovementData error = pathFinder.getEncoderPath();
            double[] speeds = calculateDrivePowers(Vmax, goal);//PIDFDrive.calculateDrivePowers(Vmax, error);
            setMotorPowers(speeds);
            switch (which) {
                case 0:
                    if (Math.abs(goal.getX() - activeLocation.getFieldX()) < 40) {
                        arrived = true;
                    }
                    break;
                case 1:
                    if (Math.abs(goal.getY() - activeLocation.getFieldY()) < 40) {
                        arrived = true;
                    }
                    break;
                case 2:
                    if (Math.abs(pathFinder.getEncoderPath().getAngleInDegrees()) <= 25) {
                        arrived = true;
                    }
                    break;
                default:
                    if ((Math.abs(goal.getX() - activeLocation.getFieldX()) < 40) && (Math.abs(goal.getY() - activeLocation.getFieldY()) < 40) && (Math.abs(pathFinder.getEncoderPath().getAngleInDegrees()) <= 25)) {
                        arrived = true;
                    }
            }

        }
        return arrived;
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
