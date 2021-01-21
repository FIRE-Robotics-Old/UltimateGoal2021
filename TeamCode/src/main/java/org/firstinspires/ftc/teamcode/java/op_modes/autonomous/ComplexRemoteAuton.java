package org.firstinspires.ftc.teamcode.java.op_modes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
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
        public RevColorSensorV3 colorSensor;
        int movement = 0;
        int red = 0;

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
            colorSensor = hardwareMap.get(RevColorSensorV3.class,"colorSensor");
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)colorSensor).enableLight(true);
            }

            PIDFDrive = new PIDFController(0.0011844, 0.000000, 0.00150719, 0);
            PIDFStrafe = new PIDFController(0.001705, 0.000000, 0.005705, 0);
            PIDFTurn = new PIDFController(0.35, 0.00000, 0.395, 0);

            autoDriving = new AutoDriving(PIDFDrive, PIDFStrafe, PIDFTurn, robot);


            telemetry.addData("Status", "Initialized");
            telemetry.update();


            waitForStart();
            runtime.reset();
            boolean end = false;


            try {

                if (opModeIsActive() && !isStopRequested()) {
                    autoDriving.setStartLocation(1850.85,0,90);
                    autoDriving.stopAt(MovementData.withDegrees(1550,892.8,90),.3);
                    red = colorSensor.red();
                    if (red > 200){
                        telemetry.speak("Zone Owen");
                        autoDriving.stopAt(MovementData.withDegrees(1850,2900,0),.3);
                    }else if (red > 55){
                        telemetry.speak("Zone Bri");
                        autoDriving.stopAt(MovementData.withDegrees(1400,2500,0),.3);
                        sleep(1000);
                    }else{
                        telemetry.speak("Zone Daniel");
                        sleep(1000);
                        autoDriving.stopAt(MovementData.withDegrees(1850,1900,0),.3);
                    }
                    autoDriving.stopAt(MovementData.withDegrees(1850,2070,0),.3);

                    end = true;

                }
                if (runtime.milliseconds() >= 29000 || end) {
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