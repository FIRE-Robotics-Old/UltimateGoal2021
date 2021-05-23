package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
//import org.firstinspires.ftc.teamcode.java.movement.AutoAdjusting;
//import org.firstinspires.ftc.teamcode.java.movement.AutoAdjusting;
import org.firstinspires.ftc.teamcode.java.util.*;

@TeleOp(name = "Final TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private DcMotor intakeAndDelivery;
    private DcMotorEx rightShooter;
    private DcMotorEx leftShooter;
    //public RevColorSensorV3 colorSensor;
//    private DcMotor elevator;
    //private TouchSensor wobbleDetector;
    //private TouchSensor ringCounter;


    RobotHardware robot = new RobotHardware();

    // TODO change the max speed to 1

    private double maxSpeed = 1;


    private double drive = 0;
    private double strafe = 0;
    private double twist = 0;
    private double shooterPower = 0;


    private boolean ifReversedIntakePressed = false;
    private final boolean shooterIsPressed = false;
    private boolean slowModePressed = false;
    private boolean slowMode = false;


   // private AutoAdjusting autoAdjusting;
    private ActiveLocation activeLocation;
    Thread locationThread;

    int red;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        frontLeftMotor = robot.frontLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backLeftMotor = robot.backLeftMotor;
        backRightMotor = robot.backRightMotor;
        rightShooter = robot.rightShooter;
        leftShooter = robot.leftShooter;

        //rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

//        elevator = hardwareMap.get(DcMotor.class, "Elevator");
//        elevator.setDirection(DcMotor.Direction.FORWARD);
        //elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //colorSensor = hardwareMap.get(RevColorSensorV3.class,"colorSensor");
       intakeAndDelivery = robot.intakeAndDelivery;
        rightShooter = robot.rightShooter;
        leftShooter = robot.leftShooter;
        //lowerWobble =robot.lowerWobble;
        //wobbleDetector =robot.wobbleDetector;
        //ringCounter =robot.ringCounter;


        activeLocation = new ActiveLocation(robot);
        locationThread = new Thread(activeLocation);
        locationThread.start();
        //autoAdjusting = new AutoAdjusting(robot);

        waitForStart();
        try {
            //TODO mode change and angel reset and shooter adjusting

            while (opModeIsActive()) {
                //motors powers calculation

                drive = -gamepad1.left_stick_y * Math.cos(activeLocation.getAngleInRadians()) +
                        gamepad1.left_stick_x * Math.sin(activeLocation.getAngleInRadians());
                strafe = gamepad1.left_stick_x * Math.cos(activeLocation.getAngleInRadians()) -
                        -gamepad1.left_stick_y * Math.sin(activeLocation.getAngleInRadians());
                twist = gamepad1.right_stick_x;

                // wheel speed calculation
                double[] speeds = {
                        (drive + strafe + twist),
                        (drive - strafe - twist),
                        (drive - strafe + twist),
                        (drive + strafe - twist)
                };

                // Finds the max after converting doubles to Doubles
                double max = Math.abs(speeds[0]);
                for (double speed : speeds) {
                    if (Math.abs(speed) > max) {
                        max = Math.abs(speed);
                    }
                }
                //double max = Math.abs(Collections.max(Arrays.stream(speeds).boxed().collect(Collectors.toList())));

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
                    maxSpeed = 1;
                } else if (!gamepad1.a) {
                    slowModePressed = false;
                }
//                if (gamepad1.x) {
//                    turnTo(Constants.backAngle);
//                }
                //Adjust spe


                shooterPower = gamepad2.right_trigger;

                // Angle Resetting
                if (gamepad1.start) {
                    activeLocation.resetAngle();
                }


                // reversing the intake and delivery
                if (gamepad2.back && !ifReversedIntakePressed) {
                    intakeAndDelivery.setDirection(DcMotorSimple.Direction.FORWARD);
                    ifReversedIntakePressed = true;
                } else if (!gamepad2.back) {
                    ifReversedIntakePressed = false;
                    intakeAndDelivery.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                //setting the speed to the motors
                frontLeftMotor.setPower(speeds[0]);
                frontRightMotor.setPower(speeds[1]);
                backLeftMotor.setPower(speeds[2]);
                backRightMotor.setPower(speeds[3]);
                intakeAndDelivery.setPower(gamepad2.left_trigger);

                leftShooter.setPower(shooterPower);
                 rightShooter.setPower(shooterPower);
                //telemetry.addData("Red", red);
//                telemetry.addData("FL", frontLeftMotor.getPower());
//                telemetry.addData("FR", frontRightMotor.getPower());
//                telemetry.addData("BL", backLeftMotor.getPower());
//                telemetry.addData("BR", backRightMotor.getPower());
                //telemetry.addData("Angle", activeLocation.getAngleInDegrees());
                //telemetry.addData("Shooter Power", rightShooter.getPower());
                //telemetry.addData("sPAIN", activeLocation.getAngleInDegrees());
                telemetry.addData("V", leftShooter.getVelocity(AngleUnit.RADIANS));
                telemetry.addData("X:", strafe);
                telemetry.addData("y", drive);
                telemetry.addData("a", twist);
                telemetry.addData("angle:", activeLocation.getAngleInDegrees());
                telemetry.addData("max speed", maxSpeed);
                telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            telemetry.update();
            sleep(2000);
            activeLocation.stop();
            requestOpModeStop();
        }
    }
//    public void turnTo(double angle) {
//        if (Math.abs(activeLocation.getAngleInDegrees() - angle) < 1) {
//            return;
//        }
//        //double maxTime = runtime.milliseconds()+5000;
//        int direct = 1;
//        double power = 0.1;
//        double aToMove = angle - activeLocation.getAngleInDegrees();
////        if (aToMove > Math.PI) {
////            aToMove = -(direct - aToMove);
////        } else if (aToMove < -Math.PI) {
////            aToMove = -(TAU - Math.abs(aToMove));
////        }
//        while ((Math.abs(activeLocation.getAngleInDegrees() - angle) > 0.5) /*&& runtime.milliseconds()<maxTime*/) {
//            //double aToMove = Math.abs(AL.getAngleInDegrees()-angle);
//            aToMove = angle - activeLocation.getAngleInDegrees();
//            if (aToMove > 180) {
//                direct = 1;
//                power = minPow + ppd * (360 - aToMove);
//            } else if (aToMove < -180) {
//                direct = -1;
//                power = minPow + ppd * (360 + aToMove);
//            } else {
//                direct = (int) -(Math.abs(aToMove) / aToMove);
//                power = minPow + ppd * (Math.abs(aToMove));
//            }
//            frontRightMotor.setPower(-0.3 * direct);
//            frontLeftMotor.setPower(0.3 * direct);
//            backRightMotor.setPower(-0.3 * direct);
//            backLeftMotor.setPower(0.3 * direct);
//            telemetry.addData("Angles:", "" + activeLocation.getAngleInDegrees() + ":" + aToMove);
//            telemetry.update();
//        }
//    }
}
