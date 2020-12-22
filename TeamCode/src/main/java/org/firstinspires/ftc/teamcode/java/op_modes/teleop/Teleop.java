package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.java.fieldmapping.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.utils.Hardware;

@TeleOp(name = "Final TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor intakeDelivery;
    public DcMotor leftShooter;
    public DcMotor rightShooter;

    Hardware robot;

    // TODO change the max speed to 1

    double maxSpeed = 0.5;
    double drive = 0;
    double strafe = 0;
    double twist = 0;
    double intakeAndDeliveryPower = 0;
    double shooterPower = 0;

    boolean ifReversedIntakePressed = false;
    boolean shooterIsPressed = false;
    boolean slowModePressed = false;
    boolean slowMode = false;

    ActiveLocation activeLocation;
    Thread locationThread;

    @Override
    public void runOpMode() {
        frontLeft = robot.frontLeftMotor;
        frontRight = robot.frontRightMotor;
        backLeft = robot.backLeftMotor;
        backLeft = robot.backRightMotor;

        activeLocation = new ActiveLocation(robot);
        locationThread = new Thread(activeLocation);
        locationThread.start();

        waitForStart();
        //TODO mode change and angel reset and shooter adjusting

        while (opModeIsActive()) {
            //motors powers calculation
            drive = gamepad1.left_stick_y * Math.cos(activeLocation.getAngle()) -
                    gamepad1.left_stick_x * Math.sin(activeLocation.getAngle());
            strafe = gamepad1.left_stick_x * Math.cos(activeLocation.getAngle()) +
                    gamepad1.left_stick_y * Math.sin(activeLocation.getAngle());
            twist = gamepad1.right_stick_x;
            intakeAndDeliveryPower = gamepad2.left_trigger;

            // wheel speed calculation
            double[] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            //finding the biggest drive motor power
            double max = Math.abs((speeds[0]));
            for (double speed : speeds) {
                if (max < Math.abs(speed)) max = Math.abs(speed);
            }

            //setting the max speed while keeping the ratio
            // change the number to change the max speed (0-1)

            //TODO: This turns everything scaling speed to 1, not max speed. Quick Fix by Multiplying by max speed
            if (max > maxSpeed) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            // slow mode
            if (gamepad1.a && !slowMode && !slowModePressed) {
                slowModePressed = true;
                slowMode = true;
                maxSpeed = 0.3;
            } else if (gamepad1.a && slowMode && !slowModePressed) {
                slowModePressed = true;
                slowMode = false;
                maxSpeed = 0.5;
            } else if (!gamepad1.a) {
                slowModePressed = false;
            }

            // Angle Resetting
            if (gamepad1.start) {
                activeLocation.resetAngle();
            }

            // reversing the intake and delivery
            if (gamepad2.back && !ifReversedIntakePressed) {
                intakeAndDeliveryPower *= -1;
                ifReversedIntakePressed = true;
            } else if (!gamepad2.back) {
                ifReversedIntakePressed = false;
            }

            //turning on the shooter

            if (gamepad2.a && !shooterIsPressed) {
                shooterIsPressed = true;
                shooterPower = 1;
            } else if (!gamepad2.a) {
                shooterIsPressed = false;
                shooterPower = 0;
            }

            //setting the speed to the motors
            frontLeft.setPower(speeds[0]);
            frontRight.setPower(speeds[1]);
            backLeft.setPower(speeds[2]);
            backRight.setPower(speeds[3]);
            intakeDelivery.setPower(intakeAndDeliveryPower);
            leftShooter.setPower(shooterPower);
            rightShooter.setPower(shooterPower);
        }

    }
}
