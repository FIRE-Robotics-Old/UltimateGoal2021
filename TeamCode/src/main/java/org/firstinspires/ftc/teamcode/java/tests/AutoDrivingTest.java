package org.firstinspires.ftc.teamcode.java.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.fieldmapping.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.fieldmapping.AutoDriving;
import org.firstinspires.ftc.teamcode.java.fieldmapping.PathFinder;
import org.firstinspires.ftc.teamcode.java.utils.MovementData;
import org.firstinspires.ftc.teamcode.java.utils.PIDFController;
import org.firstinspires.ftc.teamcode.java.utils.RobotHardware;

//To fix error perhaps flip the switch

@Autonomous(name = "AutoDrivingTest", group = "Backup")

public class AutoDrivingTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    // Declare OpMode members.
    RobotHardware robot = new RobotHardware();
    //Hardware robot = new Hardware();
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private BNO055IMU imu;
    private ActiveLocation AL;
    private Thread locationThread;
    private PathFinder PF;
    private Thread pathThread;
    private AutoDriving autoDriving;
    private PIDFController PIDF;

    //private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        imu = robot.imu;

        frontLeftMotor = robot.frontLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor = robot.backRightMotor;
        backLeftMotor = robot.backLeftMotor;


        AL = new ActiveLocation(robot);
        locationThread = new Thread(AL);
        locationThread.start();

        PIDF = new PIDFController(.5, 0, 0, 0);
        autoDriving = new AutoDriving(PIDF);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        try {
            telemetry.speak("World");
            telemetry.update();
            sleep(1000);
            AL.setStartPosition(0, 0);
            //PF.setDestination(0,100);
            telemetry.speak("Hello");
            telemetry.update();
            sleep(1000);
            /*
            telemetry.addData("FL", frontLeftMotor.getCurrentPosition());
            telemetry.addData("BR", backRightMotor.getCurrentPosition());
            telemetry.addData("FR", frontRightMotor.getCurrentPosition());
            telemetry.addData("BL", backLeftMotor.getCurrentPosition());
            telemetry.update();
        }*/

            telemetry.speak("Hello World");
            telemetry.update();
            sleep(1000);
            telemetry.addData("X", AL.getFieldX());
            telemetry.addData("Y", AL.getFieldY());
            telemetry.addData("Angle", AL.getAngle());
            telemetry.addData("Path: ", PF.getEncoderPath());
            telemetry.update();
            sleep(3000);

            autoDriving.stopAt(new MovementData(0, 100, 0), .3);
            telemetry.speak("Hello");
            telemetry.update();
            sleep(1000);

        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            telemetry.update();
            sleep(3000);
        }
    }
}