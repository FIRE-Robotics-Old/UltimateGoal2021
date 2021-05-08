package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.arcrobotics.ftclib.files.hardware.motors.Motor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.java.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.movement.AutoAdjusting;
import org.firstinspires.ftc.teamcode.java.movement.AutoDrivingNew;
import org.firstinspires.ftc.teamcode.java.util.Angle;
import org.firstinspires.ftc.teamcode.java.util.Constants;
import org.firstinspires.ftc.teamcode.java.util.MovementData;
import org.firstinspires.ftc.teamcode.java.util.PidfConstants;
import org.firstinspires.ftc.teamcode.java.util.PidfController;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;
import org.firstinspires.ftc.teamcode.java.util.Side;

@TeleOp(name="The Real TeleOp", group="TeleOp")
public class TeleopNew extends LinearOpMode {

	/**
	 * The Maximum Power Allocated to a Motor
	 */
	static double maxSpeed = 0.9;
	private final ElapsedTime runtime = new ElapsedTime();
	private double minPow = 0.15;
	private double maxPow = 0.3;
	private double ppd = (maxPow-minPow)/180;
	private double slowdist = 1000; //600
	private AutoDrivingNew autoDriving;
	//private double ppdst = (maxPow-minPow)/slowdist;
	//private double startdist = 150;
	//private double powerFator = 0.3;
	private PidfController PIDFDrive;
	private PidfController PIDFStrafe;
	private PidfController PIDFTurn;
	@Override
	public void runOpMode() throws InterruptedException {
		// Create an Instance of the Robot

		RobotHardware robot = new RobotHardware();
		robot.init(hardwareMap);

		// Create Movement Motors and Initialize
		DcMotorEx frontLeftMotor = robot.frontLeftMotor;
		DcMotorEx frontRightMotor = robot.frontRightMotor;
		DcMotorEx backLeftMotor = robot.backLeftMotor;
		DcMotorEx backRightMotor = robot.backRightMotor;

		// Create Shooter and Delivery Motors and Initialize
		DcMotorEx leftShooter = robot.leftShooter;
		DcMotorEx intakeAndDelivery = (DcMotorEx) robot.intakeAndDelivery;
		Servo ringArm = hardwareMap.get(Servo.class, "ringArm");
		Servo ringGrabber = hardwareMap.get(Servo.class, "ringGrabber");
		Servo toDelivery = hardwareMap.get(Servo.class, "toDelivery");
		Servo lowerWobble = robot.lowerWobble;

		ActiveLocation activeLocation = new ActiveLocation(robot);
		Thread activeLocationThread = new Thread(activeLocation);
		activeLocationThread.start();

		PIDFDrive = new PidfController(0,0,0,0);
		PIDFStrafe = new PidfController(0,0,0,0);
		PIDFTurn = PidfConstants.USTurn;


		autoDriving = new AutoDrivingNew(PIDFDrive, PIDFStrafe, PIDFTurn, robot, telemetry);
		autoDriving.telemetry = telemetry;

        //AutoAdjusting adjuster = new AutoAdjusting(robot, activeLocation, Side.RED, PidfConstants.USTurn, telemetry);

		MecanumDrive robotDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

		boolean locked = false;
		boolean slowMode = false;
		boolean slowModePressed = false;
		autoDriving.setDefaultErrorRanges(new MovementData(100000, 1000000, Angle.fromDegrees(5, false)));
		waitForStart();
		while (opModeIsActive()) {
			//if (gamepad1.x) locked = !locked;
			double drive = -gamepad1.left_stick_y * Math.cos(activeLocation.getAngleInRadians()) +
					gamepad1.left_stick_x * Math.sin(activeLocation.getAngleInRadians());
			double strafe = gamepad1.left_stick_x * Math.cos(activeLocation.getAngleInRadians()) -
					-gamepad1.left_stick_y * Math.sin(activeLocation.getAngleInRadians());
			//double turn = locked ? adjuster.getTurnPower() : gamepad1.right_stick_x;
			double turn = gamepad1.right_stick_x;

			double[] speeds = MecanumDrive.calculateDrivePowers(drive, strafe, turn);
			double max = Math.abs(speeds[0]);
			for (int i = 1; i < speeds.length; i++) {
				max = Math.max(max, Math.abs(speeds[i]));
			}

			if (max > maxSpeed) {
				for (int i = 0; i < speeds.length; i++) {
					speeds[i] *= maxSpeed / max;
				}
			}

			// Slow Mode
			if (gamepad1.a) slowMode = !slowMode;

			if (slowMode) {
				maxSpeed = 0.2;
			} else {
				maxSpeed = 0.9;
			}
			if (gamepad1.x) {
				frontLeftMotor.setPower(0);
				frontRightMotor.setPower(0);
				backLeftMotor.setPower(0);
				backRightMotor.setPower(0);
				telemetry.speak("X");
				sleep(100);
//				boolean hit = autoDriving.stopAt(new MovementData(0, 0, Angle.fromDegrees(-20, false)), 0.9, new MovementData(9000000, 9000000, Angle.fromDegrees(5, false)),4000);
//				if (hit){
//					telemetry.speak("hit");
//				}
//				telemetry.speak("Over");
//				sleep(100);

				sleep(100);
				if (Math.abs(activeLocation.getAngleInDegrees() + 19) < 1) {
					return;
				}
				double maxTime = runtime.milliseconds() + 3000;
				int direct = 1;
				double power = 0.1;
				double aToMove = activeLocation.getAngleInDegrees()+19;
//        if (aToMove > Math.PI) {
//            aToMove = -(direct - aToMove);
//        } else if (aToMove < -Math.PI) {
//            aToMove = -(TAU - Math.abs(aToMove));
//        }
				while (Math.abs(activeLocation.getAngleInDegrees() + 19) > 3) /*&& runtime.milliseconds()<maxTime*/ {
					//double aToMove = Math.abs(AL.getAngleInDegrees()-angle);
					//(360+353)%360
					aToMove = (360+activeLocation.getAngleInDegrees())%360-19;
					if (aToMove >0) {
						direct = 1;
						power = minPow + ppd * (360 - aToMove);
					} else if (aToMove < 0) {
						direct = -1;
						power = minPow + ppd * (360 + aToMove);
					} else {
						direct = (int) (Math.abs(aToMove) / aToMove);
						power = minPow + ppd * (Math.abs(aToMove));
					}

					frontRightMotor.setPower(-power * direct);
					frontLeftMotor.setPower(power * direct);
					backRightMotor.setPower(-power * direct);
					backLeftMotor.setPower(power * direct);
					sleep(100);
				}
			}

			// Adjust Speed (DPAD UP ⇒ +0.1, down ⇒ -0.1)
//			maxSpeed += (gamepad1.dpad_up ? 0.1 : gamepad1.dpad_down ? -0.1 : 0);
//			// Keeps it between 0.3 and 0.9
//			maxSpeed = Math.min(Math.max(maxSpeed, 0.3), 0.9);
			if (gamepad1.dpad_up && maxSpeed < 0.9) {
				maxSpeed += 0.1;
			}
			if (gamepad1.dpad_down && maxSpeed > 0.3) {
				maxSpeed -= 0.1;
			}

			// Lower Wobble
			if (gamepad1.right_bumper) {
				if (lowerWobble.getPosition() >= (Math.abs(Constants.lowerWobbleUp - .2))) {
					lowerWobble.setPosition(Constants.lowerWobbleDown);
				} else {
					lowerWobble.setPosition(Constants.lowerWobbleUp);
				}
			}

			if (gamepad2.b) {
				intakeAndDelivery.setPower(0);
			}
			else if (gamepad2.dpad_right && intakeAndDelivery.getPower() < 0.9) {
				intakeAndDelivery.setPower(intakeAndDelivery.getPower() + 0.3);
			}
			else if (gamepad2.dpad_left && intakeAndDelivery.getPower() > -0.8) {
				intakeAndDelivery.setPower(intakeAndDelivery.getPower() - 0.3);
			}

			if (gamepad2.x) {
				leftShooter.setVelocity(17.1, AngleUnit.RADIANS);
			}
			else if (gamepad2.a) {
				leftShooter.setPower(0);
			}

			if (gamepad2.dpad_up) {
				double currentPower = leftShooter.getVelocity(AngleUnit.RADIANS);
				if (currentPower <= 20) {
					currentPower += 0.10;
					leftShooter.setVelocity(currentPower, AngleUnit.RADIANS);
				}
			}
			else if (gamepad2.dpad_down) {
				double currentPower = leftShooter.getVelocity(AngleUnit.RADIANS);
				if (currentPower >= 5) {
					currentPower -= 0.10;
					leftShooter.setVelocity(currentPower, AngleUnit.RADIANS);
				}
			}

			if (gamepad2.right_bumper){
				ringGrabber.setPosition(0.3);
			}
			else if (gamepad2.right_trigger>.1){
				ringGrabber.setPosition(0);
			}

			if (gamepad2.left_bumper){
				toDelivery.setPosition(0.8);
			}
			else if (gamepad2.left_trigger>.1){
				toDelivery.setPosition(0);
			}


//			if (gamepad2.right_bumper && ringGrabber.getPosition() >= .1) {
//				ringGrabber.setPosition(0);
//				//telemetry.speak("Zero");
//			} else if (gamepad2.right_bumper && ringGrabber.getPosition() <= 0.4) {
//				ringGrabber.setPosition(0.3);
//			}

//			if (gamepad2.left_bumper && toDelivery.getPosition() > 0.2) {
//				telemetry.addData("info", toDelivery.getPosition());
//				telemetry.update();
//				toDelivery.setPosition(0.1);
//			} else if (gamepad2.left_bumper && toDelivery.getPosition() <= 0.2) {
//				toDelivery.setPosition(0.8);
//			}


			if (gamepad1.start) {
				activeLocation.resetAngle();
				autoDriving.setStartLocation(new MovementData(0, 0, Angle.fromDegrees(0)));

			}

			robotDrive.driveWithPower(speeds[0], speeds[1], speeds[2], speeds[3]);
			sleep(250);
			telemetry.addData("speed", maxSpeed);
			telemetry.addData("V", leftShooter.getVelocity(AngleUnit.RADIANS));
			telemetry.addData("A", activeLocation.getAngleInDegrees());
			telemetry.update();
		}
	}
}
