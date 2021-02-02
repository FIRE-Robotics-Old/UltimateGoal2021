package org.firstinspires.ftc.teamcode.java.util;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;

/**
 * Allows the automatic adjustment of the Robot towards the shooter
 */
public class AutoAdjusting implements Runnable {

	private final static double shooterLength = 420.69; // Temporary Value
	private final static double velocityGoal  = 420.69; // Temporary Value

	private final RobotHardware robot;
	private final AnalogInput potentiometer;
	private final ActiveLocation activeLocation;

	public AutoAdjusting(RobotHardware robot, ActiveLocation activeLocation) {
		this.robot = robot;
		this.activeLocation = activeLocation;
		potentiometer = robot.potentiometer;
	}

	/**
	 * adjusting the pitch angle (using PIDF)
	 */
	public void adjustPitch(Side side, Goal goal) {

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
	 *    R
	 * <p>
	 * Calculates the angle between the Robot and the Goal and returns the Necessary Adjustment
	 */
	public void adjustYaw(Side side, Goal goal) {
		double currentAngle  = activeLocation.getAngle();
		double currentFieldX = activeLocation.getFieldX();
		double currentFieldY = activeLocation.getFieldY();

		GoalPosition goalData = GoalPosition.generate(side, goal);
		double goalFieldX = goalData.xPosition;
		double goalFieldY = goalData.yPosition;

		double deltaX = goalFieldX- currentFieldX;
		double deltaY = goalFieldY - currentFieldY;
	}

	public double getShooterPitchAngle() {
		return (potentiometer.getVoltage() * 81.8);
	}

	private double getHeight() {
		return (
				shooterLength * Math.sin(getShooterPitchAngle())+ // Height caused by Shoot
				+
						er
				)
	}

	private boolean isRunning() {
		return true;
	}

	@Override
	public void run() {
		//this.robot.init(hardwareMap);
		while (isRunning()) {
			// Do Something
		}
	}
}