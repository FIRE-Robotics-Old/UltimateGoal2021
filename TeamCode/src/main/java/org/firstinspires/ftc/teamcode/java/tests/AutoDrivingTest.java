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
    private PIDFController PIDFDrive;
    private PIDFController PIDFStrafe;
    private PIDFController PIDFTurn;

    private boolean location;

    //private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        location = true;
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
        //12096
        //PIDFDrive = new PIDFController(0.0011844/*96004999*/, 0.000000, 0.00150719/*423*/, 0); //003,000001,003705
        PIDFDrive = new PIDFController(0.00, 0.000000, 0.00, 0);
        PIDFStrafe = new PIDFController(0.000, 0.000000, 0.0000, 0);
        PIDFTurn = new PIDFController(0.35, 0.00000, 0.36, 0);

        autoDriving = new AutoDriving(PIDFDrive, PIDFStrafe, PIDFTurn, robot);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();
        boolean stat = false;
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
            //telemetry.update();

            while (opModeIsActive() && !isStopRequested()) {
                //frontLeftMotor.setPower(.29);


                if (!stat) {
                    location = true;
                    String report = autoDriving.stopAt(MovementData.withDegrees(0, 0, 90), .3);
                    //String report = autoDriving.errorReport(MovementData.withDegrees(0, 600, 0));
                    telemetry.addData("Error Report", report);
                    telemetry.update();
                }

                //String report = autoDriving.errorReport(new MovementData(600, 600, 90));
                //telemetry.addData("Error Report", report);
                //telemetry.speak("Hello" + stat);
            /*telemetry.addData("Angle", PIDF.getAngleErrorDegrees());
            telemetry.addData("FL", frontLeftMotor.getPower());
            telemetry.addData("FR", frontRightMotor.getPower());
            telemetry.addData("BL", backLeftMotor.getPower());
            telemetry.addData("BR", backRightMotor.getPower());

             */
                //telemetry.update();
                //sleep(200);

                if (runtime.milliseconds() >= 29000 || stat) {
                    location = false;
                    frontRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    String report = autoDriving.errorReport(MovementData.withDegrees(0, 600, 0));
                    telemetry.addData("Error Report", report);
                    //frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    telemetry.speak("Done");
                    telemetry.update();
                    //AL.Stop();
                    //PF.stop();

                }

            }
            //AL.Stop();
            //PF.stop();
        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            telemetry.addData("bob", location);
            telemetry.update();
            //AL.Stop();
            //PF.stop();
            sleep(10000);
        }
        //}
    }
}