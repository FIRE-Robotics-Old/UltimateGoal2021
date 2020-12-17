package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.fieldmapping.ActiveLocation;
import org.firstinspires.ftc.teamcode.fieldmapping.PathFinder;

import java.nio.file.Path;

//To fix error perhaps flip the switch

@Autonomous(name="ActiveLocationTest", group="Backup")

public class ActiveLocationTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Hardware robot = new Hardware();
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private BNO055IMU imu;
    private ActiveLocation AL;
    private Thread locationThread;
    private PathFinder PF;
    private Thread pathThread;
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

        PF = new PathFinder(AL);
        pathThread = new Thread(PF);
        pathThread.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        try{
            AL.setStartPosition(0,0);
            PF.setDestination(0,100);
            /*
            telemetry.addData("FL", frontLeftMotor.getCurrentPosition());
            telemetry.addData("BR", backRightMotor.getCurrentPosition());
            telemetry.addData("FR", frontRightMotor.getCurrentPosition());
            telemetry.addData("BL", backLeftMotor.getCurrentPosition());
            telemetry.update();
        }*/
            double startTime = runtime.milliseconds();
            double currentTime = 0;
            while (opModeIsActive()) {
                telemetry.addData("X", AL.getFieldX());
                telemetry.addData("Y", AL.getFieldY());
                telemetry.addData("Angle", AL.getAngle());
                telemetry.addData("Path: ", PF.getEncoderPath());
                telemetry.update();
                currentTime = runtime.milliseconds();
                sleep(100);
            }
        }catch (Exception e){
            telemetry.addData("error:",e.getStackTrace());
        }
    }
}