package org.firstinspires.ftc.teamcode.java.op_modes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.java.util.RobotHardware;
//import org.firstinspires.ftc.teamcode.java.fieldmapping.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.util.*;


@Autonomous(name="BasicRemoteAuton", group="Backup")
public class BasicRemoteAuton extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private Servo wobbleGrip;
    private Servo wobble2;
    private BNO055IMU imu;
    private ActiveLocation AL;
    private Thread locationThread;
    public RevColorSensorV3 colorSensor;
    private int red = 0;
    private double open = .9;
    private double close = .5;
    private double angleGoal;
    private double yGoal;

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

        wobbleGrip = hardwareMap.get(Servo.class, "wobbleGrip");
        wobble2 = hardwareMap.get(Servo.class, "wobble2");

        colorSensor = hardwareMap.get(RevColorSensorV3.class,"colorSensor");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

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
            sleep(1000);
            wobble2.setPosition(0.5);

            double startTime = runtime.milliseconds();
            double currentTime = 0;

            AL.setStartPosition(0, 0,0);
            telemetry.addData("FL:", AL.getAngleInDegrees());
            telemetry.update();
            //moveY(600);//1193
            angleGoal = 0;
            yGoal = AL.getFieldY()-150;
            moveX(900);
            sleep(1000);
            //turn(0);
            adjustErrorY(angleGoal,yGoal);
            sleep(100);
            red = colorSensor.red();
            sleep(150);
            telemetry.speak("Red"+red);
            sleep(1000);
            telemetry.update();
            if (red < 300) {
                telemetry.addData("Zone", "A");
                telemetry.update();
                /*moveX(2032);
                sleep(100);
                moveY(-200);
                sleep(100);
                //moveX(2332);
                sleep(100);
                 */
                moveX(1650);

            /*else if (red >= 48 && red < 300){
                telemetry.speak("Zone B");
                telemetry.update();
                moveX(2032);
                adjustErrorY(0,0);
                moveY(200);
                moveX(1650);
                adjustErrorY(0,200);
            }*/
            }else{
                telemetry.addData("Zone","C");
                telemetry.update();
                moveX(1650);
                adjustErrorY(0,0);
                moveX(2590);
                adjustErrorY(0,0);
                sleep(1000);
                moveX(1650);
            }
            wobbleGrip.setPosition(.9);
/*
            telemetry.addData("BL",backLeftMotor.getCurrentPosition());
            telemetry.addData("FR", frontRightMotor.getCurrentPosition());
            telemetry.addData("Y", AL.getFieldY());
            telemetry.addData("X", AL.getFieldX());
            telemetry.update();
            off();
            sleep(5000);

/*
            while (AL.getFieldY() > 1193) {
                frontLeftMotor.setPower(0.3);
                frontRightMotor.setPower(0.3);
                backLeftMotor.setPower(0.3);
                backRightMotor.setPower(0.3);
                telemetry.addData("In Loop: ", 1);
                telemetry.update();
                currentTime = runtime.milliseconds();
            }*/
            //telemetry.addData("In Loop: ", 0);
            /*

            telemetry.update();

            //wobbleGrip.setPosition(open);
            moveX(1980);
            //wobbleGrip.setPosition(close);

            off();
            AL.setStop();


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
*/
            telemetry.update();
            stop();

        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            AL.stop();
        }
    }
    public void moveY(double y){
        int direct;
        while (Math.abs(AL.getFieldY()-y)>5){
            if (AL.getFieldY()>y){
                direct =-1;
            } else{
                direct = 1;
            }
            synchronized (this) {
                frontLeftMotor.setPower(0.3 * direct);
                frontRightMotor.setPower(0.3 * direct);
                backLeftMotor.setPower(0.3 * direct);
                backRightMotor.setPower(0.3 * direct);
            }
            telemetry.addData("In Loop: ",  AL.getFieldY());
            telemetry.update();
            output("Y"+y,AL.getFieldY());
        }
        off();
    }
    public void moveX(double x){
        int direct;
        while (Math.abs(AL.getFieldX()-x)>50){
            if (AL.getFieldX()>x){
                direct =-1;
            } else{
                direct = 1;
            }
            synchronized (this) {
                frontLeftMotor.setPower(-0.3 * direct);
                frontRightMotor.setPower(0.3 * direct);
                backLeftMotor.setPower(0.3 * direct);
                backRightMotor.setPower(-0.3 * direct);
            }
            telemetry.addData("In Loop: ",  AL.getFieldX());
            telemetry.update();
            output("X"+x,AL.getFieldX());
        }
        off();
    }
    public void turn(double angle){
        double maxTime = runtime.milliseconds()+5000;
        int direct = 1;
        if (AL.getAngleInDegrees()<angle){
            direct = -1;
        }
        while (AL.getAngleInDegrees() < angle && runtime.milliseconds()<maxTime){
            frontRightMotor.setPower(-0.3*direct);
            frontLeftMotor.setPower(-0.3*direct);
            backRightMotor.setPower(0.3*direct);
            backLeftMotor.setPower(0.3*direct);
            output("A"+angle,AL.getAngleInDegrees());
        }
        off();
    }
    public void adjustErrorY(double angle, double Y){
        if (Math.abs(AL.getAngleInDegrees()-angle)>5){
            turn(angleGoal);
        }
        sleep(1000);
        /*if (Math.abs(AL.getFieldYAbs()-Y)>1){
            telemetry.addData("Moving Y", AL.getFieldY());
            telemetry.update();
            moveY(Y);
        }
*/
    }
    public void output(String label, double val){
        telemetry.addData(label,val);
        telemetry.update();
    }

    public void off(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

}