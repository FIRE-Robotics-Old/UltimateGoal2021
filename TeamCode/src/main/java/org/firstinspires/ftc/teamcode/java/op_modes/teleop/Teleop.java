package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.java.fieldmapping.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.utils.AutoAdjusting;
import org.firstinspires.ftc.teamcode.java.utils.Hardware;

@TeleOp(name = "Final TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private DcMotor intakeAndDelivery;
    private DcMotor leftShooter;
    private DcMotor rightShooter;
    private Servo lowerWobble;
    private TouchSensor wobbleDetector;
    private TouchSensor ringCounter;

    Hardware robot = new Hardware();

    // TODO change the max speed to 1

    private double maxSpeed = 0.5;


    private double drive = 0;
    private double strafe = 0;
    private double twist = 0;
    private double intakeAndDeliveryPower = 0;
    private double shooterPower = 0;
    private int rings =0;

    private boolean ifReversedIntakePressed = false;
    private boolean shooterIsPressed = false;
    private boolean slowModePressed = false;
    private boolean slowMode = false;


    private AutoAdjusting autoAdjusting;
    private ActiveLocation activeLocation;
    Thread locationThread;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        frontLeftMotor = robot.frontLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backLeftMotor = robot.backLeftMotor;
        backRightMotor = robot.backRightMotor;
        intakeAndDelivery = robot.intakeAndDelivery;
        rightShooter = robot.rightShooter;
        leftShooter = robot.leftShooter;
        lowerWobble =robot.lowerWobble;
        wobbleDetector =robot.wobbleDetector;
        ringCounter =robot.ringCounter;


        activeLocation = new ActiveLocation(robot);
        locationThread = new Thread(activeLocation);
        locationThread.start();
        autoAdjusting = new AutoAdjusting(robot);

        waitForStart();
        try {
            //TODO mode change and angel reset and shooter adjusting

            while (opModeIsActive()) {
                //motors powers calculation
                drive = gamepad1.left_stick_y * Math.cos(activeLocation.getAngle()) -
                        gamepad1.left_stick_x * Math.sin(activeLocation.getAngle());
                strafe = gamepad1.left_stick_x * Math.cos(activeLocation.getAngle()) +
                        gamepad1.left_stick_y * Math.sin(activeLocation.getAngle());
                twist = gamepad1.right_stick_x;


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
                    for (int i = 0; i < speeds.length; i++) speeds[i] /= (1/maxSpeed)*max;
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
                if (rings<3){
                    intakeAndDeliveryPower = gamepad2.left_trigger;
                    if (ringCounter.isPressed()){
                        rings++;
                    }
                }else if (gamepad2.right_bumper)
                {
                    rings=0;
                }else {
                    intakeAndDeliveryPower=0;
                }

                // Angle Resetting
                if (gamepad1.start) {
                    activeLocation.resetAngle();
                }

                // reversing the intake and delivery
                if (gamepad2.back && !ifReversedIntakePressed) {
                    intakeAndDelivery.setDirection(DcMotorSimple.Direction.REVERSE);
                    ifReversedIntakePressed = true;
                } else if (!gamepad2.back) {
                    ifReversedIntakePressed = false;
                    intakeAndDelivery.setDirection(DcMotorSimple.Direction.FORWARD);
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
                frontLeftMotor.setPower(speeds[0]);
                frontRightMotor.setPower(speeds[1]);
                backLeftMotor.setPower(speeds[2]);
                backRightMotor.setPower(speeds[3]);
                intakeAndDelivery.setPower(intakeAndDeliveryPower);
                leftShooter.setPower(shooterPower);
                rightShooter.setPower(shooterPower);
                telemetry.addData("field X:",activeLocation.getFieldX());
                telemetry.addData("field Y:",activeLocation.getFieldY());
                telemetry.addData("potentiometer",autoAdjusting.getShooterPitchAngle());
                telemetry.addData("angle:",activeLocation.getAngleInDegrees());
                telemetry.addData("rings",rings);
                telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            activeLocation.setStop();
        }
    }
}
