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
import org.firstinspires.ftc.teamcode.java.movement.AutoAdjusting;
import org.firstinspires.ftc.teamcode.java.util.*;

@TeleOp(name = "Final TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private DcMotor intakeAndDelivery;
    //private DcMotorEx rightShooter;
    private DcMotorEx leftShooter;
    //public RevColorSensorV3 colorSensor;
//    private DcMotor elevator;
    private Servo ringArm;

    private Servo lowerWobble;
    private Servo ringGrabber;
    private Servo toDelivery;
    //private TouchSensor wobbleDetector;
    //private TouchSensor ringCounter;

    private double minPow = 0.15;
    private double maxPow = 0.8;
    private double ppd = (maxPow-minPow)/180;
    private double slowdist = 1000; //600
    private double ppdst = (maxPow-minPow)/slowdist;
    private double startdist = 150;
    private double powerFator = 0.3;

    RobotHardware robot = new RobotHardware();

    // TODO change the max speed to 1

    private double maxSpeed = 0.7;


    private double drive = 0;
    private double strafe = 0;
    private double twist = 0;
    //private final double intakeAndDeliveryPower = 0;
    private final double shooterPower = 0;
    private final int rings = 0;

    private final boolean ifReversedIntakePressed = false;
    private final boolean shooterIsPressed = false;
    private boolean slowModePressed = false;
    private boolean slowMode = false;


    private AutoAdjusting autoAdjusting;
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
        //rightShooter = robot.rightShooter;
        leftShooter = robot.leftShooter;

        //rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

//        elevator = hardwareMap.get(DcMotor.class, "Elevator");
//        elevator.setDirection(DcMotor.Direction.FORWARD);
        //elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ringArm = hardwareMap.get(Servo.class, "ringArm");
        ringGrabber = hardwareMap.get(Servo.class, "ringGrabber");
        toDelivery = hardwareMap.get(Servo.class, "toDelivery");


        //colorSensor = hardwareMap.get(RevColorSensorV3.class,"colorSensor");
       intakeAndDelivery = robot.intakeAndDelivery;
        //rightShooter = robot.rightShooter;
        //leftShooter = robot.leftShooter;
        lowerWobble =robot.lowerWobble;
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
                    maxSpeed = 0.3;
                } else if (gamepad1.a && slowMode && !slowModePressed) {
                    slowModePressed = true;
                    slowMode = false;
                    maxSpeed = 0.7;
                } else if (!gamepad1.a) {
                    slowModePressed = false;
                }
//                if (gamepad1.x) {
//                    turnTo(Constants.backAngle);
//                }
                //Adjust speed
                if (gamepad1.dpad_up && maxSpeed < 0.9) {
                    maxSpeed += 0.1;
                }
                if (gamepad1.dpad_down && maxSpeed > 0.3) {
                    maxSpeed -= 0.1;
                }
                //Lower wobble movement
                if (gamepad1.right_bumper) {
                    if (lowerWobble.getPosition() >= (Math.abs(Constants.lowerWobbleUp - .2))) {
                        lowerWobble.setPosition(Constants.lowerWobbleDown);
                    } else {
                        lowerWobble.setPosition(Constants.lowerWobbleUp);
                    }

                }
                if (gamepad2.b) {
                    intakeAndDelivery.setPower(0);
                }
                if (gamepad2.dpad_right && intakeAndDelivery.getPower() < 0.9) {
                    intakeAndDelivery.setPower(intakeAndDelivery.getPower() + 0.3);
                }
                if (gamepad2.dpad_left && intakeAndDelivery.getPower() > -0.8) {
                    intakeAndDelivery.setPower(intakeAndDelivery.getPower() - 0.3);
                }

//                if (gamepad2.left_trigger>0.1){
//                    intakeAndDelivery.setPower(-gamepad2.left_trigger);
//                    //sleep(3000);
//                }
//                if (gamepad2.left_bumper){
//                    intakeAndDelivery.setPower(0);
//                }
                if (gamepad2.x) {
                    //rightShooter.setPower(0.85); //0.75 - 0.80
                    //rightShooter.setVelocity(20, AngleUnit.RADIANS);
                    leftShooter.setVelocity(20, AngleUnit.RADIANS);

                }
                if (gamepad2.a) {
                    //rightShooter.setPower(0);
                    leftShooter.setPower(0);

                }
                if (gamepad2.dpad_up) {
                    double currentPower = leftShooter.getVelocity(AngleUnit.RADIANS);
                    if (currentPower <= 20) {
                        currentPower += 1;
                        leftShooter.setVelocity(currentPower, AngleUnit.RADIANS);
                    }
                }
                if (gamepad2.dpad_down) {
                    double currentPower = leftShooter.getVelocity(AngleUnit.RADIANS);
                    if (currentPower >= 5) {
                        currentPower -= 1;
                        leftShooter.setVelocity(currentPower, AngleUnit.RADIANS);
                    }
                }
//              if (gamepad2.right_trigger > 0.1)
//                {
//                    //rightShooter.setPower(gamepad2.right_trigger);
//                    leftShooter.setPower(gamepad2.right_trigger);
//                }
                if (gamepad2.right_bumper && ringGrabber.getPosition() >= .1) {
                    ringGrabber.setPosition(0);
                    telemetry.speak("Zero");
                } else if (gamepad2.right_bumper && ringGrabber.getPosition() <= 0.4) {
//                    if (toDelivery.getPosition() > 0.7) {
//                        ringGrabber.setPosition(0.1);
//                    } else {
                        ringGrabber.setPosition(0.3);
                    //}
                }
                if (gamepad2.left_bumper && toDelivery.getPosition() > 0.2) {
                    telemetry.addData("info", toDelivery.getPosition());
                    telemetry.update();
                    toDelivery.setPosition(0.1);
                } else if (gamepad2.left_bumper && toDelivery.getPosition() <= 0.2) {
                    toDelivery.setPosition(0.8);
                }

                //else
                //{
                //rightShooter.setPower(0);
                //leftShooter.setPower(0);
                //}
                //if (gamepad2.b){
                //intakeAndDelivery.setPower(0);
                // }

//                red = colorSensor.red();
//
//                if (red > 200){
//                    telemetry.speak("Zone Owen");
//                }else if (red > 55){
//                    telemetry.speak("Zone Bri");
//                }else{
//                    telemetry.speak("Zone Daniel");
//                }
                //sleep(1000);

//                if (rings<3){
//                    intakeAndDeliveryPower = gamepad2.left_trigger;
//                    if (ringCounter.isPressed()){
//                        rings++;
//                    }
//                }else if (gamepad2.right_bumper)
//                {
//                    rings=0;
//                }else {
//                    intakeAndDeliveryPower=0;
//                }


                // Angle Resetting
                if (gamepad1.start) {
                    activeLocation.resetAngle();
                }

                /*
                // reversing the intake and delivery
                if (gamepad2.back && !ifReversedIntakePressed) {
                    intakeAndDelivery.setDirection(DcMotorSimple.Direction.REVERSE);
                    ifReversedIntakePressed = true;
                } else if (!gamepad2.back) {
                    ifReversedIntakePressed = false;
                    intakeAndDelivery.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                telemetry.speak("I am g");
                telemetry.update();
                sleep(1000);
                //turning on the shooter

                if (gamepad2.a && !shooterIsPressed) {
                    shooterIsPressed = true;
                    shooterPower = 1;
                } else if (!gamepad2.a) {
                    shooterIsPressed = false;
                    shooterPower = 0;
                }
                telemetry.speak("I am root");
                telemetry.update();
                sleep(1000);
                */
//                if (gamepad1.dpad_up){
//                    elevator.setPower(0.3);
//                    telemetry.speak("Yes");
//                }else if (gamepad1.dpad_down){
//                    elevator.setPower(-0.3);
//                }
//                else{
//                    elevator.setPower(0);
//                }
//                if (gamepad2.left_bumper){
//                    if (ringArm.getPosition()>.4){
//                        ringArm.setPosition(0);
//                    }else{
//                        ringArm.setPosition(1);
//                    }
//                }
                //setting the speed to the motors
                frontLeftMotor.setPower(speeds[0]);
                frontRightMotor.setPower(speeds[1]);
                backLeftMotor.setPower(speeds[2]);
                backRightMotor.setPower(speeds[3]);
                //intakeAndDelivery.setPower(intakeAndDeliveryPower);

                //leftShooter.setPower(shooterPower);
                // rightShooter.setPower(shooterPower);
                //telemetry.addData("Red", red);
//                telemetry.addData("FL", frontLeftMotor.getPower());
//                telemetry.addData("FR", frontRightMotor.getPower());
//                telemetry.addData("BL", backLeftMotor.getPower());
//                telemetry.addData("BR", backRightMotor.getPower());
                //telemetry.addData("Angle", activeLocation.getAngleInDegrees());
                //telemetry.addData("Shooter Power", rightShooter.getPower());
                //telemetry.addData("sPAIN", activeLocation.getAngleInDegrees());
                telemetry.addData("V", leftShooter.getVelocity(AngleUnit.RADIANS));
                telemetry.update();
                sleep(250);//Helps with lag
                //telemetry.addData("field X:", activeLocation.getFieldX());
                //telemetry.addData("field Y:", activeLocation.getFieldY());
                //telemetry.addData("potentiometer", autoAdjusting.getShooterPitchAngle());
                //telemetry.addData("angle:", activeLocation.getAngleInDegrees());
                //telemetry.addData("rings", rings);
                //telemetry.update();
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
