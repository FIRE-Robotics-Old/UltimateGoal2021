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


        /*AL = new ActiveLocation(robot);
        locationThread = new Thread(AL);
        locationThread.start();

        PF = new PathFinder(AL);
        pathThread = new Thread(PF);
        pathThread.start();

         */

        PIDF = new PIDFController(.75, 0.007, 0.25, 0);
        autoDriving = new AutoDriving(PIDF, robot);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        //while (opModeIsActive()) {
        // run until the end of the match (driver presses STOP)
        try {

            //AL.setStartPosition(0, 0);
            //PF.setDestination(600,600);
            /*
            telemetry.addData("FL", frontLeftMotor.getCurrentPosition());
            telemetry.addData("BR", backRightMotor.getCurrentPosition());
            telemetry.addData("FR", frontRightMotor.getCurrentPosition());
            telemetry.addData("BL", backLeftMotor.getCurrentPosition());
            telemetry.update();
        }*/
            //telemetry.addData("X", AL.getFieldX());
            //telemetry.addData("Y", AL.getFieldY());
            //telemetry.addData("Angle", AL.getAngle());
            //telemetry.addData("Path: ", PF.getEncoderPath());
            telemetry.update();
            boolean stat = autoDriving.stopAt(new MovementData(600, 600, 90), .3);
            telemetry.addData("Angle", PIDF.getAngleErrorDegrees());
            telemetry.speak("Hello" + stat);
            /*telemetry.addData("Angle", PIDF.getAngleErrorDegrees());
            telemetry.addData("FL", frontLeftMotor.getPower());
            telemetry.addData("FR", frontRightMotor.getPower());
            telemetry.addData("BL", backLeftMotor.getPower());
            telemetry.addData("BR", backRightMotor.getPower());

             */
            telemetry.update();
            sleep(4000);
            if (runtime.milliseconds() >= 2900 || stat) {
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);
            }


        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            telemetry.update();
            sleep(3000);
        }
        //}
    }
}