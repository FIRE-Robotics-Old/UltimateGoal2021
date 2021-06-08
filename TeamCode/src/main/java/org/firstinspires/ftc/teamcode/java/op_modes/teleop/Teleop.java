package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
//import org.firstinspires.ftc.teamcode.java.movement.AutoAdjusting;
//import org.firstinspires.ftc.teamcode.java.movement.AutoAdjusting;
import org.firstinspires.ftc.teamcode.java.util.*;

@TeleOp(name = "Final TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {

    //public RevColorSensorV3 colorSensor;
//    private DcMotor elevator;
    //private TouchSensor wobbleDetector;
    //private TouchSensor ringCounter;


    RobotHardware robot = new RobotHardware();


    private double maxSpeed = 1;


    private double liftPower = 0;

    private boolean isGripped = false;
    private boolean isGriperPressed = false;


    private boolean slowModePressed = false;
    private boolean slowMode = false;


    Thread locationThread;

    int red;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        DcMotor frontLeftMotor = robot.frontLeftMotor;
        DcMotor frontRightMotor = robot.frontRightMotor;
        DcMotor backLeftMotor = robot.backLeftMotor;
        DcMotor backRightMotor = robot.backRightMotor;
        DcMotorEx rightShooter = robot.rightShooter;
        DcMotorEx leftShooter = robot.leftShooter;

        DcMotor intakeAndDelivery = robot.intakeAndDelivery;

        CRServo wobbleLift1 = robot.wobbleLift1;
        CRServo wobbleLift2 = robot.wobbleLift2;
        Servo wobbleGrip = robot.wobbleGrip;


        // private AutoAdjusting autoAdjusting;
        ActiveLocation activeLocation = new ActiveLocation(robot);
        locationThread = new Thread(activeLocation);
        locationThread.start();
        //autoAdjusting = new AutoAdjusting(robot);

        waitForStart();
        try {
            //TODO mode change and angel reset and shooter adjusting

            while (opModeIsActive()) {
                //motors powers calculation

                double drive = gamepad1.left_stick_x * Math.cos(activeLocation.getAngleInRadians()) +
                        gamepad1.left_stick_y * Math.sin(activeLocation.getAngleInRadians());
                double strafe = gamepad1.left_stick_y * Math.cos(activeLocation.getAngleInRadians()) -
                        gamepad1.left_stick_x * Math.sin(activeLocation.getAngleInRadians());
                double twist = gamepad1.right_stick_x;

                // wheel speed calculation
                double[] speeds = {
                        (drive + strafe + twist),
                        (drive - strafe - twist),
                        (drive - strafe + twist),
                        (drive + strafe - twist)
                };

                double max = Math.abs(speeds[0]);
                for (double speed : speeds) {
                    if (Math.abs(speed) > max) {
                        max = Math.abs(speed);
                    }
                }

                if (max > maxSpeed) {
                    for (int i = 0; i < speeds.length; i++) speeds[i] *= maxSpeed / max;
                }

                // slow mode
                if (gamepad1.a && !slowMode && !slowModePressed) {
                    slowModePressed = true;
                    slowMode = true;
                    maxSpeed = 0.4;
                } else if (gamepad1.a && slowMode && !slowModePressed) {
                    slowModePressed = true;
                    slowMode = false;
                    maxSpeed = 0.1;
                } else if (!gamepad1.a) {
                    slowModePressed = false;
                }
                //speed adjustment
                if (gamepad1.dpad_up && maxSpeed <= 1 ) {
                    maxSpeed += 0.1;
                }
                if (gamepad1.dpad_down && maxSpeed >= 0.3) {
                    maxSpeed -= 0.1;
                }
                //wobble lift control
                if (gamepad2.dpad_up) {
                    liftPower = 1;
                }
                else if (gamepad2.dpad_up) {
                    liftPower = -1;
                }
                else {
                    liftPower = 0;
                }
                //wobble gripper control
                if (gamepad2.a && !isGripped && !isGriperPressed) {
                    wobbleGrip.setPosition(0);
                    isGripped = true;
                    isGriperPressed = true;
                } else if (gamepad2.a && isGripped && !isGriperPressed) {
                    wobbleGrip.setPosition(1);
                    isGripped = false;
                    isGriperPressed = true;
                }    else if (!gamepad2.a) {
                    isGriperPressed = false;
                }
                //shooter control TODO yew auto
                double shooterPower;
                if (gamepad2.x) {
                    shooterPower = (1);
                } else{
                    shooterPower = -gamepad2.right_trigger;
                }


                //rest angle TODO fix because it doesn't work
                if (gamepad1.start) {
                    activeLocation.resetAngle();
                }
                //intake control
                double intakeAndDeliveryPower;
                if (gamepad2.y) {
                    intakeAndDeliveryPower = (1);
                } else {
                    intakeAndDeliveryPower = -gamepad2.left_trigger;
                }
                //setting the speed to the motors
                frontLeftMotor.setPower(speeds[0]);
                frontRightMotor.setPower(speeds[1]);
                backLeftMotor.setPower(speeds[2]);
                backRightMotor.setPower(speeds[3]);

                intakeAndDelivery.setPower(intakeAndDeliveryPower);

                leftShooter.setPower(shooterPower);
                rightShooter.setPower(shooterPower);

                wobbleLift1.setPower(liftPower);
                wobbleLift2.setPower(liftPower);
            }
        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            telemetry.update();
            activeLocation.stop();
            requestOpModeStop();
        }
    }
}
