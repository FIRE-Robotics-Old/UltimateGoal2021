package org.firstinspires.ftc.teamcode.java.op_modes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.*;
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
import org.firstinspires.ftc.teamcode.java.vision.RingHeightPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

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
	long sleepTime = 500;

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
		//heightDetector.startStreaming();
		//autoDriving = new AutoDriving(PIDFDrive, PIDFStrafe, PIDFTurn,robot);

		RingHeightPipeline pipeline = new RingHeightPipeline(telemetry, false);

		OpenCvInternalCamera camera;
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"cameraMonitorViewId",
				"id",
				hardwareMap.appContext.getPackageName()
		);
		camera = OpenCvCameraFactory.getInstance().createInternalCamera(
				OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId
		);

		camera.setPipeline(pipeline);

		camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

		camera.openCameraDeviceAsync(() ->
				camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
		);
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		runtime.reset();
		int movement = 0;

		//while (opModeIsActive()) {
		// run until the end of the match (driver presses STOP)
		try {
			RingHeightPipeline.Height position = pipeline.getHeight();
			camera.stopStreaming();
			telemetry.addData("Position of Ring", position);
			telemetry.update();
			autoDriving.setStartLocation(new MovementData(0, 0, Angle.fromDegrees(0)));
			autoDriving.setDefaultErrorRanges(new MovementData(70, 140, Angle.fromDegrees(7, false)));
			while (opModeIsActive() && !isStopRequested()) {
//				leftShooter.setVelocity(17.2, AngleUnit.RADIANS);
//				wobbleGrip.setPosition(0);
//				autoDriving.stopAt(new MovementData(0, 230, Angle.fromDegrees(-23, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(10, false)),2000);
//				intakeAndDelivery.setPower(Constants.deliveryPower);
//				sleep(5000);
//				intakeAndDelivery.setPower(0);
//				leftShooter.setVelocity(0, AngleUnit.RADIANS);
//				sleep(sleepTime);
				if (runtime.milliseconds()<1700) {
					switch (position) {
						case A:
							pathA();
							telemetry.speak("Case");
							break;
						//PathC();
						case B:
							pathB();
							telemetry.speak("B");
							break;
						default:
							telemetry.speak("C");
							pathC();
					}
				}
				//pathC();
				wobbleGrip.setPosition(1);
				//autoDriving.stopAt(new MovementData(0,Constants.navLineY-100,Angle.fromDegrees(0, false)),0.9, new MovementData(70, 140, Angle.fromDegrees(10, false)),3000);


				//PATH AB
				//autoDriving.stopAt(new MovementData(-150, Constants.ALowerBorder, Angle.fromDegrees(0, false)), 0.9);

				movement += 1; //??? Might cause issue
				telemetry.update();
				stop();
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
				stop();

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


	public void pathA() {
		//autoDriving.stopAt(new MovementData(0, 230, Angle.fromDegrees(12, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(5, false)), 3000);
		//sleep(sleepTime);
		autoDriving.stopAt(new MovementData(150, Constants.ALowerBorder, Angle.fromDegrees(0, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(10, false)), 3000);
		sleep(sleepTime);
		wobbleGrip.setPosition(1);
		sleep(sleepTime);
		//autoDriving.stopAt(new MovementData(0, Constants.navLineY-200, Angle.fromDegrees(0, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(10, false)), 3000);
	}

	public void pathB() {
		autoDriving.stopAt(new MovementData(0, Constants.BLowerBorder, Angle.fromDegrees(-4, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(10, false)), 3000);
		sleep(sleepTime);
		//autoDriving.stopAt(new MovementData(-100, Constants.BLowerBorder, Angle.fromDegrees(-4, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(10, false)), 3000);
		wobbleGrip.setPosition(1);
		sleep(sleepTime);
		//autoDriving.stopAt(new MovementData(0, Constants.navLineY, Angle.fromDegrees(0, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(10, false)), 3000);
	}

	public void pathC() {
		//autoDriving.stopAt(new MovementData(100, 230, Angle.fromDegrees(5, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(5, false)), 3000);
		//sleep(sleepTime);
		autoDriving.stopAt(new MovementData(100, Constants.CLowerBorder-100, Angle.fromDegrees(0, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(10, false)), 5000);
		sleep(sleepTime);
		wobbleGrip.setPosition(1);
		sleep(sleepTime);
		autoDriving.stopAt(new MovementData(100, Constants.navLineY, Angle.fromDegrees(0, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(10, false)), 3000);
		//autoDriving.stopAt(new MovementData(0, Constants.navLineY, Angle.fromDegrees(0, false)), 0.9, new MovementData(70, 140, Angle.fromDegrees(10, false)), 3000);
	}
}
