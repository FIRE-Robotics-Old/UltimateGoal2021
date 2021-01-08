package org.firstinspires.ftc.teamcode.java.op_modes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.fieldmapping.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.utils.RobotHardware;


@Autonomous(name="BasicRemoteAuton", group="Backup")
public class BasicRemoteAuton extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private BNO055IMU imu;
    private ActiveLocation AL;
    private Thread locationThread;

//    private PathFinder PF;
//    private Thread pathThread;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        try {
            sleep(3000);
//comment

            AL.setStartPosition(304, 0);
            telemetry.addData("FL:", frontLeftMotor.getCurrentPosition());

            while (AL.getFieldY() > -1850) { //Change once fix build
                frontLeftMotor.setPower(0.3);
                frontRightMotor.setPower(0.3);
                backLeftMotor.setPower(0.3);
                backRightMotor.setPower(0.3);
                telemetry.addData("In Loop: ", 1);
                telemetry.update();
            }
            telemetry.addData("In Loop: ", 0);
            telemetry.update();
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            AL.Stop();


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
            stop();

        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            AL.Stop();
        }
    }

}