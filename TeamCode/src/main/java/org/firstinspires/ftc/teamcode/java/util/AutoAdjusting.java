package org.firstinspires.ftc.teamcode.java.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.java.util.*;

/**
 * Allows the automatic adjustment of the Robot towards the shooter
 */
public class AutoAdjusting implements Runnable {
	RobotHardware robot;
	private final AnalogInput potentiometer;

	public AutoAdjusting(RobotHardware robot) {
		this.robot = robot;
		potentiometer = robot.potentiometer;
	}

	/**
	 * adjusting the pitch angle (using PIDF)
	 */
	public void adjustPitch() {

	}

	/**
	 * adjusting the yaw angle (using PID)
	 *           X
	 *     — — — — — — — S
	 *    |
	 *    |
	 * Y  |
	 *    |
	 *    |
	 *    R
	 * <p>
	 * Calculates the angle between the Robot and the Shooter and returns the Necessary Adjustment
	 */
	public void adjustYaw() {

	}

	public double getShooterPitchAngle() {
		return (potentiometer.getVoltage() * 81.8);
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