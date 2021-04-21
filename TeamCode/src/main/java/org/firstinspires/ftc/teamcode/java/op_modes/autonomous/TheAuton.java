package org.firstinspires.ftc.teamcode.java.op_modes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptCompassCalibration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.movement.AutoDrivingNew;
import org.firstinspires.ftc.teamcode.java.util.Angle;
import org.firstinspires.ftc.teamcode.java.util.Constants;
import org.firstinspires.ftc.teamcode.java.util.MovementData;
import org.firstinspires.ftc.teamcode.java.util.PidfConstants;
import org.firstinspires.ftc.teamcode.java.util.PidfController;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;
import org.firstinspires.ftc.teamcode.java.vision.HeightDetector;

//To fix error perhaps flip the switch

@Autonomous(name = "Yes", group = "Backup")

public class TheAuton extends LinearOpMode {

	private final ElapsedTime runtime = new ElapsedTime();
	RobotHardware robot = new RobotHardware();
	private DcMotor frontRightMotor;
	private DcMotor frontLeftMotor;
	private DcMotor backLeftMotor;
	private DcMotor backRightMotor;
	private DcMotor intakeAndDelivery;
	private DcMotorEx leftShooter;
	private DcMotor elevator;
	private Servo wobbleGrip;
	private Servo ringArm;
	private Servo ringGrabber;
	private BNO055IMU imu;
	private ActiveLocation AL;
	private Thread locationThread;
	HeightDetector heightDetector;
	private AutoDrivingNew autoDriving;
	private PidfController PIDFDrive;
	private PidfController PIDFStrafe;
	private PidfController PIDFTurn;

	private boolean location;

	//private ElapsedTime runtime = new ElapsedTime();
	@Override
	public void runOpMode() {
		location = true;
		robot.init(hardwareMap);

		//heightDetector = new HeightDetector(hardwareMap, telemetry);

		imu = robot.imu;

		frontLeftMotor = robot.frontLeftMotor;
		frontRightMotor = robot.frontRightMotor;
		backRightMotor = robot.backRightMotor;
		backLeftMotor = robot.backLeftMotor;
		intakeAndDelivery = robot.intakeAndDelivery;
		leftShooter = robot.leftShooter;

		wobbleGrip = hardwareMap.get(Servo.class, "wobbleGrip");
		ringArm = hardwareMap.get(Servo.class, "ringArm");
		ringGrabber = hardwareMap.get(Servo.class, "ringGrabber");

		PIDFDrive = PidfConstants.USDrive;
		PIDFStrafe = PidfConstants.USStrafe;
		PIDFTurn = PidfConstants.USTurn;


		autoDriving = new AutoDrivingNew(PIDFDrive, PIDFStrafe, PIDFTurn, robot, telemetry);
		autoDriving.telemetry = telemetry;
		//autoDriving = new AutoDriving(PIDFDrive, PIDFStrafe, PIDFTurn,robot);
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		//heightDetector.startStreaming();
		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		runtime.reset();
		int movement = 0;

		//while (opModeIsActive()) {
		// run until the end of the match (driver presses STOP)
		try {
			autoDriving.setStartLocation(new MovementData(0, 0, Angle.fromDegrees(0)));
			autoDriving.setDefaultErrorRanges(new MovementData(70, 100, Angle.fromDegrees(10, false)));

			while (opModeIsActive() && !isStopRequested()) {
				leftShooter.setVelocity(17.1, AngleUnit.RADIANS);
				wobbleGrip.setPosition(0);
				autoDriving.stopAt(new MovementData(0,120,Angle.fromDegrees(0,false)),0.9);
				autoDriving.stopAt(new MovementData(0,120,Angle.fromDegrees(-30.886, false)),0.9);
				intakeAndDelivery.setPower(Constants.deliveryPower);
				sleep(5000);
				intakeAndDelivery.setPower(0);
				leftShooter.setVelocity(0,AngleUnit.RADIANS);

					//PATH AB
				//autoDriving.stopAt(new MovementData(-150, Constants.ALowerBorder, Angle.fromDegrees(0, false)), 0.9);
				wobbleGrip.setPosition(1);
				movement += 1; //??? Might cause issue
				telemetry.update();
				if (runtime.milliseconds() >= 29000 || movement >= 1) {
					location = false;
					frontRightMotor.setPower(0);
					frontLeftMotor.setPower(0);
					backRightMotor.setPower(0);
					backLeftMotor.setPower(0);
					telemetry.speak("Done");
					telemetry.update();
					break;
				}

			}
		} catch (Exception e) {
			telemetry.addData("error:", e.getStackTrace());
			telemetry.addData("Cause", e.getCause());
			telemetry.addData("Message", e.getMessage());
			telemetry.addData("bob", location);
			telemetry.update();
			sleep(10000);
		}
	}

}

