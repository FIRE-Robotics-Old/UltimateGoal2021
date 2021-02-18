package org.firstinspires.ftc.teamcode.java.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.movement.PathFinder;

/**
 * Allows the automatic adjustment of the Robot towards the shooter
 */
public class AutoAdjusting {

	private static final double shooterLength   = 420.69; // Temporary Value
	private static final double velocityGoal    = 420.69; // Temporary Value
	public  static final Goal initialGoal       = Goal.POWER_SHOT_1;

	private final AnalogInput       potentiometer;
	private final BNO055IMU         imu;

	private final ActiveLocation    activeLocation;
	private final PathFinder        pathFinder;
	private final Thread            pathFinderThread;

	private final PIDFController    PidfYaw, PidfPitch;


	private double defaultYawErrorRange     = 5;
	private double defaultPitchErrorRange   = 5;

	public double deltaX            = 0;
	public double deltaY            = 0;
	public double deltaZ            = 0;

	private double shooterRotationPower = 0;
	private double robotTurnPower = 0;

	private GoalPosition activeGoal;
	private int goalPositionIndex = 0;

	public AutoAdjusting(RobotHardware robot, ActiveLocation activeLocation, Side side) {
		this.potentiometer  = robot.potentiometer;
		this.imu            = robot.imu;

		this.activeLocation = activeLocation;

		this.pathFinder         = new PathFinder(activeLocation);
		this.pathFinderThread   = new Thread(pathFinder);
		pathFinderThread.start();

		// TODO: Calibrate turn and Pitch Values
		this.PidfYaw    = new PIDFController(0.35, 0.00000, 0.395, 0);
		this.PidfPitch  = new PIDFController(0,0,0,0);

		activeGoal = GoalPosition.generate(side, initialGoal);
	}

	private void update() {
		deltaX = activeGoal.xPosition - activeLocation.getFieldX();
		deltaY = activeGoal.yPosition - activeLocation.getFieldY();
		deltaZ = activeGoal.height    - getCurrentHeight();
	}

	public void cycleRight() {
		goalPositionIndex = activeGoal.side == Side.RED
				? (goalPositionIndex + 1) % Goal.values().length
				: (goalPositionIndex + Goal.values().length - 1) % Goal.values().length;
	}

	public void cycleLeft() {
		goalPositionIndex = activeGoal.side == Side.BLUE
				? (goalPositionIndex + 1) % Goal.values().length
				: (goalPositionIndex + Goal.values().length - 1) % Goal.values().length;
	}

	public AutoAdjusting(RobotHardware robot, ActiveLocation activeLocation, Side side, double angleErrorRange) {
		this(robot, activeLocation, side);
	}

	/**
	 * adjusting the pitch angle (using PIDF)
	 */
	public void calculatePitchPower() {
		shooterRotationPower = PidfPitch.calculatePID(deltaZ);
	}

	/**
	 * adjusting the yaw angle (using PID)
	 * <p>
	 *           X
	 *     — — — — — — — S
	 *    |
	 *    |
	 * Y  |
	 *    |
	 *    |
	 *    |
	 * <p>
	 * Calculates the angle between the Robot and the Goal and returns the Necessary Adjustment
	 */
	public void calculateYawPower() {
		double currentAngle = activeLocation.getAngle();
		double yaw          = Math.atan2(deltaX, deltaY);
		double error        = currentAngle - yaw;
		robotTurnPower      = PidfYaw.calculatePID(error);
	}

	public double getShooterPitchAngle() {
		return (potentiometer.getVoltage() * 81.8);
	}

	private double getCurrentHeight() {
		update();
		double distance = Math.sqrt(squared(deltaX) + squared(deltaY));
		double theta = getShooterPitchAngle();
		return  Math.tan(theta) * distance
				- imu.getGravity().zAccel * distance / (2 * squared(velocityGoal) * squared(Math.cos(theta))) // TODO: Figure out IMU Axis
				+ shooterLength * Math.sin(theta);
	}

	public double getTurnPower() {
		calculateYawPower();
		return robotTurnPower;
	}

	public double getShooterTurnPower(){
		calculatePitchPower();
		return shooterRotationPower;
	}

	private boolean isRunning() {
		return true;
	}

	private double squared(double num) {
		return num * num;
	}

	private int squared(int num) {
		return num * num;
	}
}