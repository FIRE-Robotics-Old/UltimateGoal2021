package org.firstinspires.ftc.teamcode.java.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.java.util.MovementData;
import org.firstinspires.ftc.teamcode.java.util.Vector2d;

public class MecanumDrive extends Drivetrain {

	enum MotorPosition {
		frontLeft(0),
		frontRight(1),
		backLeft(2),
		backRight(3);

		int position;

		MotorPosition(int position) {
			this.position = position;
		}
	}

	DcMotor[] motors;

	private double angleOffset = 0;

	public MecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
		motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
	}

	public MecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, double angleOffset) {
		this(frontLeft, frontRight, backLeft, backRight);
		this.angleOffset = angleOffset;
	}

	public void setAngleOffset(double angleOffset) {
		this.angleOffset = angleOffset;
	}

	/**
	 * Stop the motors
	 */
	@Override
	public void stop() {
		for (DcMotor motor : motors) {
			motor.setPower(0);
		}
	}

	private void driveWithPower(double frontLeft, double frontRight, double backLeft, double backRight) {
		motors[MotorPosition.frontLeft.position].setPower(frontLeft);
		motors[MotorPosition.frontRight.position].setPower(frontRight);
		motors[MotorPosition.backLeft.position].setPower(backLeft);
		motors[MotorPosition.backRight.position].setPower(backRight);
	}

	public void drive(double driveSpeed, double strafeSpeed, double turnSpeed) {
		driveWithPower(
				driveSpeed + strafeSpeed + turnSpeed,
				driveSpeed - strafeSpeed - turnSpeed,
				driveSpeed - strafeSpeed + turnSpeed,
				driveSpeed + strafeSpeed - turnSpeed
		);
	}

	public static double[] calculateDrivePowers(double driveSpeed, double strafeSpeed, double turnSpeed) {
		return new double[] {
				driveSpeed + strafeSpeed + turnSpeed,
				driveSpeed - strafeSpeed - turnSpeed,
				driveSpeed - strafeSpeed + turnSpeed,
				driveSpeed + strafeSpeed - turnSpeed
		};
	}

	public void drive(double driveSpeed, double strafeSpeed, double turnSpeed, boolean quadraticGrowth) {
		if (quadraticGrowth) {
			driveSpeed  = squareInput(driveSpeed);
			strafeSpeed = squareInput(strafeSpeed);
			turnSpeed   = squareInput(turnSpeed);
		}

		drive(driveSpeed, strafeSpeed, turnSpeed);
	}


	public void driveFieldOriented(double driveSpeed, double strafeSpeed, double turnSpeed, double gyroAngle) {
		Vector2d translation = new Vector2d(strafeSpeed, driveSpeed);
		translation = translation.rotateBy(gyroAngle); //Maybe need to make negative angles

		double theta = translation.angle();

		double[] wheelSpeeds = new double[4];

		int flp = MotorPosition.frontLeft.position;
		int frp = MotorPosition.frontRight.position;
		int blp = MotorPosition.backLeft.position;
		int brp = MotorPosition.backRight.position;

		wheelSpeeds[flp] = Math.sin(theta + PI4);
		wheelSpeeds[frp] = Math.sin(theta - PI4);
		wheelSpeeds[blp] = Math.sin(theta - PI4);
		wheelSpeeds[brp] = Math.sin(theta + PI4);

		scaleSpeeds(wheelSpeeds, translation.magnitude());

		wheelSpeeds[flp] += turnSpeed;
		wheelSpeeds[frp] -= turnSpeed;
		wheelSpeeds[blp] -= turnSpeed;
		wheelSpeeds[brp] += turnSpeed;

		scaleSpeeds(wheelSpeeds);

		motors[flp].setPower(wheelSpeeds[flp] * maxSpeed);
		motors[frp].setPower(wheelSpeeds[frp] * maxSpeed);
		motors[blp].setPower(wheelSpeeds[blp] * maxSpeed);
		motors[brp].setPower(wheelSpeeds[brp] * maxSpeed);
	}

	public void driveFieldOriented(double driveSpeed, double strafeSpeed, double turnSpeed, double gyroAngle, boolean quadraticGrowth) {
		if (quadraticGrowth) {
			driveSpeed  = squareInput(driveSpeed);
			strafeSpeed = squareInput(strafeSpeed);
			turnSpeed   = squareInput(turnSpeed);
		}

		driveFieldOriented(driveSpeed, strafeSpeed, turnSpeed, gyroAngle);
	}
}
