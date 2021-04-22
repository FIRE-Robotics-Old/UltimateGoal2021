package org.firstinspires.ftc.teamcode.java.movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.java.util.Angle;
import org.firstinspires.ftc.teamcode.java.util.Goal;
import org.firstinspires.ftc.teamcode.java.util.GoalPosition;
import org.firstinspires.ftc.teamcode.java.util.PidfController;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;
import org.firstinspires.ftc.teamcode.java.util.Side;

import static org.firstinspires.ftc.teamcode.java.util.MathUtil.squared;

/**
 * Allows the automatic adjustment of the Robot towards the shooter
 */
public class AutoAdjusting {

	private static final double shooterLength = 420.69; // Temporary Value
	private static final double velocityGoal = 420.69; // Temporary Value
	private static final double angleOffset = 45; // TODO: change it to the real angle offset
	private static final double maxAngle = 45; // TODO: chang to the angle
	public static final Goal initialGoal = Goal.POWER_SHOT_1;

	private final AnalogInput potentiometer;
	private final BNO055IMU imu;

	private final ActiveLocation activeLocation;

	private final PidfController PidfYaw, PidfPitch;

	public double deltaX = 0;
	public double deltaY = 0;
	public double deltaZ = 0;

	private double shooterRotationPower = 0;
	private double robotTurnPower = 0;

	private final Side side;
	private GoalPosition activeGoal;
	private int goalPositionIndex = 0;

	Telemetry telemetry;

	public AutoAdjusting(RobotHardware robot, ActiveLocation activeLocation, Side side) {
		this.potentiometer = robot.potentiometer;
		this.imu = robot.imu;

		this.activeLocation = activeLocation;

		// TODO: Calibrate turn and Pitch Values
		this.PidfYaw = new PidfController(0.35, 0.00000, 0.395, 0);
		this.PidfPitch = new PidfController(0, 0, 0, 0);

		this.side = side;
		activeGoal = GoalPosition.generate(side, initialGoal);
	}

	public AutoAdjusting(RobotHardware robot, ActiveLocation activeLocation, Side side, PidfController turnController, Telemetry telemetry) {
		this.potentiometer = robot.potentiometer;
		this.imu = robot.imu;
		this.telemetry = telemetry;

		this.activeLocation = activeLocation;

		// TODO: Calibrate turn and Pitch Values
		this.PidfYaw = turnController;
		this.PidfPitch = new PidfController(0, 0, 0, 0);

		this.side = side;
		activeGoal = GoalPosition.generate(side, initialGoal);
	}

	private void update() {
		deltaX = activeGoal.xPosition - activeLocation.getFieldX();
		deltaY = activeGoal.yPosition - activeLocation.getFieldY();
		deltaZ = activeGoal.height - getCurrentHeight();
	}

	public void cycleRight() {
		goalPositionIndex =
				activeGoal.side == Side.RED ? (goalPositionIndex + 1) % Goal.values().length
						: (goalPositionIndex + Goal.values().length - 1) % Goal.values().length;
		activeGoal = GoalPosition.generate(side, Goal.values()[goalPositionIndex]);
	}

	public void cycleLeft() {
		goalPositionIndex =
				activeGoal.side == Side.BLUE ? (goalPositionIndex + 1) % Goal.values().length
						: (goalPositionIndex + Goal.values().length - 1) % Goal.values().length;
		activeGoal = GoalPosition.generate(side, Goal.values()[goalPositionIndex]);
	}

	/**
	 * adjusting the pitch angle (using PIDF)
	 */
	public void calculatePitchPower() {
		shooterRotationPower = PidfPitch.calculate(deltaZ);
	}

	/**
	 * adjusting the yaw angle (using PID)
	 * <p>
	 * X — — — — — — — S | | Y | | | |
	 * <p>
	 * Calculates the angle between the Robot and the Goal and returns the Necessary Adjustment
	 */
	public void calculateYawPower() {
		double currentAngle = activeLocation.getAngleInRadians();
		// TODO: Offset
		double yaw = Math.atan2(deltaX, deltaY) - Math.toRadians(-19);
		double error = currentAngle - yaw;
		if (telemetry != null) {
			telemetry.addData("Delta X", deltaX);
			telemetry.addData("Delta Y", deltaY);
			telemetry.addData("Angle Error", error);
			telemetry.addData("Yaw Angle", yaw);
			telemetry.update();
		}
		robotTurnPower = PidfYaw.calculate(error);
	}

	public double getShooterPitchAngle() {
		return (potentiometer.getVoltage() * 81.8) + angleOffset;
	}

	private double getCurrentHeight() {
		update();
		double distance = Math.sqrt(squared(deltaX) + squared(deltaY));
		double theta = getShooterPitchAngle();
		return Math.tan(theta) * distance
				- imu.getGravity().zAccel * distance
				// TODO: Figure out IMU Axis
				/ (2 * squared(velocityGoal) * squared(Math.cos(theta)))
				+ shooterLength * Math.sin(theta);
	}



	public double getTurnPower() {
		calculateYawPower();
		return robotTurnPower;
	}

	/**
	 * adjusting the yaw angle (using PID)
	 * <p>
	 * X — — — — — — — S | | Y | | | |
	 * <p>
	 * Calculates the angle between the Robot and the Goal and returns the Necessary Adjustment
	 */
	public void calculateYawPower(Angle errorAngle, AutoDrivingNew driving) {
		double currentAngle = activeLocation.getAngleInRadians();
		double yaw = Math.atan2(deltaX, deltaY);
		double error = currentAngle - yaw + Math.toRadians(18.5);
		if (telemetry != null) {
			telemetry.addData("Delta X", deltaX);
			telemetry.addData("Delta Y", deltaY);
			telemetry.addData("Angle Error", error);
			telemetry.addData("Yaw Angle", yaw);
			telemetry.update();
		}

		driving.rotateTo(Angle.fromRadians(error), 0.9, errorAngle);
	}

	public double getTurnPower(Angle angleError, AutoDrivingNew driving) {
		calculateYawPower(angleError, driving);
		return robotTurnPower;
	}

	public double getShooterTurnPower() {

		if (shooterRotationPower > 0 && getShooterPitchAngle() >= maxAngle) {
			return 0;
		} // safety check
		return shooterRotationPower;
	}
}
