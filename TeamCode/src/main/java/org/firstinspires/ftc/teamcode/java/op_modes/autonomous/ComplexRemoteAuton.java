package org.firstinspires.ftc.teamcode.java.op_modes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.movement.AutoDriving;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.java.util.RobotHardware;
//import org.firstinspires.ftc.teamcode.java.fieldmapping.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.util.*;


    @Autonomous(name="theC00lerBasicRemoteAuton", group="auton")
    public class ComplexRemoteAuton extends LinearOpMode {

        private final ElapsedTime runtime = new ElapsedTime();
        // Declare OpMode members.
        RobotHardware robot = new RobotHardware();

        private DcMotor frontRightMotor;
        private DcMotor frontLeftMotor;
        private DcMotor backLeftMotor;
        private DcMotor backRightMotor;
        private BNO055IMU imu;
        private AutoDriving autoDriving;
        private PIDFController PIDFDrive;
        private PIDFController PIDFStrafe;
        private PIDFController PIDFTurn;
        int movement = 0;

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

            PIDFDrive = new PIDFController(0.0011844, 0.000000, 0.00150719, 0);
            PIDFStrafe = new PIDFController(0.001705, 0.000000, 0.005705, 0);
            PIDFTurn = new PIDFController(0.35, 0.00000, 0.395, 0);

            autoDriving = new AutoDriving(PIDFDrive, PIDFStrafe, PIDFTurn, robot);


            telemetry.addData("Status", "Initialized");
            telemetry.update();


            waitForStart();
            runtime.reset();
            boolean stat = false;


            try {

                if (opModeIsActive() && !isStopRequested()) {
                    //frontLeftMotor.setPower(.29);
                    autoDriving.stopAt(MovementData.withDegrees(00, 600,90), .3);
                    //autoDriving.turnOff();
                    sleep(1000);
                    autoDriving.stopAt(MovementData.withDegrees(00, 00,-90), .3);
                    //autoDriving.turnOff();
                    sleep(1000);
                    autoDriving.stopAt(MovementData.withDegrees(300, 00,-90), .3);
                    movement = 2;

                }
                if (runtime.milliseconds() >= 29000 || movement>1) {
                    location = false;
                    frontRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    telemetry.speak("Done");
                    telemetry.update();
                    requestOpModeStop();

                }
            } catch (Exception e) {
                telemetry.addData("error:", e.getStackTrace());
                telemetry.addData("bob", location);
                telemetry.update();
                sleep(10000);
            }

        }
    }