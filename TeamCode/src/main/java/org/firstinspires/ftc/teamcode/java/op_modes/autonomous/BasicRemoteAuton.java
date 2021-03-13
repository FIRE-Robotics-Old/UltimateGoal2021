package org.firstinspires.ftc.teamcode.java.op_modes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.java.util.RobotHardware;
//import org.firstinspires.ftc.teamcode.java.fieldmapping.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.util.*;
import org.firstinspires.ftc.teamcode.java.vision.HeightDetector;
import org.firstinspires.ftc.teamcode.java.vision.RingHeightPipeline;

import static org.firstinspires.ftc.teamcode.java.util.Constants.TAU;
import static org.firstinspires.ftc.teamcode.java.util.Constants.cornerAB;
import static org.firstinspires.ftc.teamcode.java.util.Constants.lowerWobbleDown;


@Autonomous(name="BasicRemoteAuton", group="Backup")
public class BasicRemoteAuton extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor elevator;
    private Servo wobbleGrip;
    private Servo ringArm;
    private BNO055IMU imu;
    private ActiveLocation AL;
    private Thread locationThread;
    HeightDetector heightDetector;
    //public RevColorSensorV3 colorSensor;
    private int red = 0;
    private double open = .9;
    private double close = .5;
    private int yDirect = 1;
    private int xDirect = 1;
    private double angleGoal;
    private double yGoal;
    private double minPow = 0.15;
    private double maxPow = 0.77;
    private double ppd = (maxPow-minPow)/180;
    private double slowdist = 900; //600
    private double ppdst = (maxPow-minPow)/slowdist;
    private double startdist = 100;
    private double powerFator = 0.3;
//    private PathFinder PF;
//    private Thread pathThread;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        heightDetector = new HeightDetector(hardwareMap, telemetry);

        imu = robot.imu;

        frontLeftMotor = robot.frontLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor = robot.backRightMotor;
        backLeftMotor = robot.backLeftMotor;

//        elevator = hardwareMap.get(DcMotor.class, "Elevator");
//	    elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	    elevator.setDirection(DcMotor.Direction.FORWARD);


        wobbleGrip = hardwareMap.get(Servo.class, "wobbleGrip");
//        ringArm = hardwareMap.get(Servo.class, "ringArm");

//        colorSensor = hardwareMap.get(RevColorSensorV3.class,"colorSensor");
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor).enableLight(true);
//        }

        AL = new ActiveLocation(robot);
        locationThread = new Thread(AL);
        locationThread.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        heightDetector.startStreaming();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        try {
            RingHeightPipeline.Height position = heightDetector.getHeight();
            heightDetector.stopStreaming();
            telemetry.addData("Position of Ring", position);
            telemetry.update();
            //sleep(1000);
            double startTime = runtime.milliseconds();
            double currentTime = 0;
            //wobbleGrip.setPosition(Constants.lowerWobbleDown);
            AL.setStartPosition(0, 0,0);

            //Path B:

	        wobbleGrip.setPosition(Constants.lowerWobbleDown);
            switch (position) {
                case A:
                    PathAB();
                case B:
                    PathAB();
                case C:
                    PathC();

            }
	        //sleep(100);
	        //moveY(Constants.navLineY);



//	        sleep(1000);
//            moveY(2000);
//            sleep(100);
//            moveX(-300);
//            wobbleGrip.setPosition(Constants.lowerWobbleUp);
//            telemetry.speak("Wobble one");
//            sleep(100);
//            //turnTo(0);
//            moveY(1780);
//            sleep(100);
//	        moveX(Constants.wobbleX);
//            sleep(100);
//            turnTo(180);
//            yDirect = -1;
//            xDirect = -1;
//            sleep(1000);
//            moveY(Constants.wobbleY+30);//30
//            sleep(100);
//            moveX(Constants.wobbleX);
//            sleep(100);
//            turnTo(180);
//			sleep(1000);
//            wobbleGrip.setPosition(Constants.lowerWobbleDown);
//            sleep(500);
//            moveX(300);
//            sleep(100);
//            turnTo(180);
//            sleep(100);
//            moveY(Constants.ALowerBorder+300);



//            moveY(Constants.BLowerBorder+350);
//            wobbleGrip.setPosition(Constants.lowerWobbleUp);
//            sleep(100);
//            moveY(Constants.navLineY-200);
            //Path C



	        //turnTo(180);
	        //moveX(-50);
	        //moveY(Constants.CLowerBorder);
	        //double moveTime = runtime.milliseconds()-startMove;
	        //telemetry.speak(""+moveTime);
//	        sleep(500);
//	        moveY(Constants.CLowerBorder-200);
//	        sleep(100);
//	        moveX(300);
//	        turnTo(180);
//	        moveY(Constants.backFieldY);
	        //elevator.setPower(0);
//	        //ringArm.setPosition();
//	        sleep(500);
//	        //ringArm.setPosition(0);
//	        moveY(Constants.backFieldY-100);
//	        //elevator.setTargetPosition(0);
//	        moveY(Constants.navLineY);

//	        if (runtime.milliseconds()+(2*moveTime)<25){
//		        sleep(100);
//		        turn(180);
//		        yDirect = -1;
//		        moveY(Constants.wobbleY);
//				wobbleGrip.setPosition(Constants.lowerWobbleDown);
//				sleep(100);
//				moveY(Constants.CLowerBorder);
//				wobbleGrip.setPosition(Constants.lowerWobbleUp);
//				sleep(100);
//	        }
	        //moveY(Constants.navLineY);







			/*
            moveY(1430); //A Y
            turn(0);
            sleep(4000);
            moveY(0);
            turn(0);
            moveY(1450);
            sleep(2000);
            moveY(1880); //parking pot

             */
            //moveY(2000);//B Y
            //moveY(2600); //zC Y
            //sleep(3000);
            //moveY(3075);//edge of field
//            sleep(2000);
//            turn(0);
//            sleep(100);
//            moveY(1880);
//            //moveX(0);
//            sleep(100);
            //turn(0);
//            switch (position) {
//                case A:
//                    moveY(1970);
//                    turn(0);
//                    moveX(0);
//                case B:
//                    moveY(2000);
//                case C:
//                    moveY(2200);
//                    turn(0);
//                    //moveX(300);
//
//            }
            //wobbleGrip.setPosition(Constants.lowerWobbleUp);
            //turn(0);
            //sleep(3000);
            //moveY(1980);
            //turn(0);
//            telemetry.addData("FL:", AL.getAngleInDegrees());
//            telemetry.update();
//            //moveY(600);//1193
//            angleGoal = 0;
//            yGoal = AL.getFieldY()-150;
//            moveX(900);
//            sleep(1000);
//            //turn(0);
//            adjustErrorY(angleGoal,yGoal);
//            sleep(100);
//            red = colorSensor.red();
//            sleep(150);
//            telemetry.speak("Red"+red);
//            sleep(1000);
//            telemetry.update();
//            if (red < 300) {
//                telemetry.addData("Zone", "A");
//                telemetry.update();
//                /*moveX(2032);
//                sleep(100);
//                moveY(-200);
//                sleep(100);
//                //moveX(2332);
//                sleep(100);
//                 */
//                moveX(1650);
//
//            /*else if (red >= 48 && red < 300){
//                telemetry.speak("Zone B");
//                telemetry.update();
//                moveX(2032);
//                adjustErrorY(0,0);
//                moveY(200);
//                moveX(1650);
//                adjustErrorY(0,200);
//            }*/
//            }else{
//                telemetry.addData("Zone","C");
//                telemetry.update();
//                moveX(1650);
//                adjustErrorY(0,0);
//                moveX(2590);
//                adjustErrorY(0,0);
//                sleep(1000);
//                moveX(1650);
//            }
//            wobbleGrip.setPosition(.9);
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
            AL.stop();

        } catch (Exception e) {
	        telemetry.addData("error:", e.getStackTrace());
	        AL.stop();
        }
    }
    public void moveY(double y){
        if (Math.abs(AL.getFieldY()-y)<10){
            return;
        }
        int direct;
        double power = maxPow;
        double odist =Math.abs(AL.getFieldY()-y);
        while (Math.abs(AL.getFieldY()-y)>20){
            if (AL.getFieldY()>y){
                direct =-1*yDirect;
            } else{
                direct = 1*yDirect;
            }
            double dist = Math.abs(AL.getFieldY()-y);
            if (odist-dist<startdist){
                power =maxPow*powerFator;
            } else if (dist >= slowdist){
                power = maxPow;
            }
            else {
                power=minPow+dist*ppdst;
            }
            synchronized (this) {
                frontLeftMotor.setPower(power * direct);
                frontRightMotor.setPower(power * direct);
                backLeftMotor.setPower(power * direct);
                backRightMotor.setPower(power * direct);
            }
            telemetry.addData("In Loop: ",  AL.getFieldY());
            telemetry.update();
            output("Y"+y,AL.getFieldY());
        }
        off();
    }
    public void moveX(double x){
        if (Math.abs(AL.getFieldX()-x)<10){
            return;
        }
        int direct;
	    double maxTime = runtime.milliseconds()+5000;
        double odist =Math.abs(AL.getFieldX()-x);
        while (Math.abs(AL.getFieldX()-x)>20 /*&& runtime.milliseconds()<maxTime*/){
            //double xToMove = AL.getFieldX()-x;
//            if (xToMove > 0){
//                direct = 1;
//            } else {
//                direct = -1;
//            }
            if (AL.getFieldX()<x){
                 direct =-1*xDirect;
            } else{
                direct = 1*xDirect;
            }
            double power = 0.3;
            double dist = Math.abs(AL.getFieldX()-x);
            if (odist-dist<startdist){
                power =maxPow*powerFator;
            } else if (dist >= slowdist){
                power = maxPow;
            }
            else {
                power=minPow+dist*ppdst;
            }
            synchronized (this) {
                frontLeftMotor.setPower(-power * direct);
                frontRightMotor.setPower(power * direct);
                backLeftMotor.setPower(power * direct);
                backRightMotor.setPower(-power * direct);
            }
            telemetry.addData("In Loop: ",  AL.getFieldX());
            telemetry.update();
            output("X"+x,AL.getFieldX());
        }
        off();
    }
    public void turnTo(double angle){
    	if (Math.abs(AL.getAngleInDegrees()-angle) <1){
    		return;
	    }
        double maxTime = runtime.milliseconds()+5000;
        int direct = 1;
        double power = 0.1;
        double aToMove = angle - AL.getAngleInDegrees();
//        if (aToMove > Math.PI) {
//            aToMove = -(direct - aToMove);
//        } else if (aToMove < -Math.PI) {
//            aToMove = -(TAU - Math.abs(aToMove));
//        }
        while ((Math.abs(AL.getAngleInDegrees()-angle)>2) /*&& runtime.milliseconds()<maxTime*/){
            //double aToMove = Math.abs(AL.getAngleInDegrees()-angle);
            aToMove = angle - AL.getAngleInDegrees();
            if (aToMove > 180) {
                direct = 1;
                power = minPow+ppd*(360-aToMove);
            } else if (aToMove < -180) {
                direct = -1;
                power = minPow+ppd*(360+aToMove);
            }else{
                direct = (int) -(Math.abs(aToMove)/aToMove);
                power = minPow+ppd*(Math.abs(aToMove));
            }
//            if (AL.getAngleInDegrees() > angle){
//                direct = 1;
//            } else {
//                direct = -1;
//            }

            frontRightMotor.setPower(-power*direct);
            frontLeftMotor.setPower(power*direct);
            backRightMotor.setPower(-power*direct);
            backLeftMotor.setPower(power*direct);
            output("A"+angle,AL.getAngleInDegrees());
            //telemetry.addData("loop", AL.getAngleInDegrees() < angle && runtime.milliseconds()<maxTime);
            //telemetry.update();
        }
        off();
    }
    @Deprecated
    public void turn(double angle){
    	int direct;
    	if (Math.abs(AL.getAngle()-angle)<3){
    		return;
	    }
	    double maxTime = runtime.milliseconds()+500;
    	double aToMove = angle - AL.getAngleInDegrees();

    	while (Math.abs(AL.getAngleInDegrees()-angle)<3 && runtime.milliseconds()<maxTime){
            if (aToMove > 180) {
                direct =-1;//(int) -((360 - aToMove)/(360 - aToMove));

            } else if (aToMove < -180) {
                direct = 1;//(int)-((360 - Math.abs(aToMove))/(360 - Math.abs(aToMove)));
            }else{
                direct = (int) (aToMove/aToMove);
            }
//    		if (angle < path){
//    			direct =1;
//		    }else{
//    			direct = -1;
//		    }
		    frontRightMotor.setPower(-0.3*direct);
		    frontLeftMotor.setPower(0.3*direct);
		    backRightMotor.setPower(-0.3*direct);
		    backLeftMotor.setPower(0.3*direct);
    		telemetry.addData("Angles:", ""+AL.getAngleInDegrees()+":"+aToMove);
    		telemetry.update();
	    }
    }
    public void adjustErrorY(double angle, double Y){
        if (Math.abs(AL.getAngleInDegrees()-angle)>5){
            turnTo(angleGoal);
        }
        sleep(1000);
        /*if (Math.abs(AL.getFieldYAbs()-Y)>1){
            telemetry.addData("Moving Y", AL.getFieldY());
            telemetry.update();
            moveY(Y);
        }
*/
    }
    public void PathAB(){
        wobbleGrip.setPosition(Constants.lowerWobbleDown);
        //sleep(100);
        //moveY(Constants.navLineY);
        sleep(1000);
        moveY(2000);
        sleep(100);
        moveX(-300);
        wobbleGrip.setPosition(Constants.lowerWobbleUp);
        telemetry.speak("Wobble one");
        sleep(100);
        //turnTo(0);
        moveY(1780);
        sleep(100);
        moveX(Constants.wobbleX);
        sleep(100);
        turnTo(180);
        yDirect = -1;
        xDirect = -1;
        sleep(1000);
        moveY(Constants.wobbleY+30);//30
        sleep(100);
        moveX(Constants.wobbleX);
        sleep(100);
        turnTo(180);
        sleep(1000);
        wobbleGrip.setPosition(Constants.lowerWobbleDown);
        sleep(500);
        moveX(300);
        sleep(100);
        turnTo(180);
        sleep(100);
        moveY(Constants.ALowerBorder+300);

    }
    public void PathC(){
        wobbleGrip.setPosition(Constants.lowerWobbleDown);
        sleep(100);
        //double startMove = runtime.milliseconds();
        //moveX(-300);
        moveY(Constants.CLowerBorder+200);
        sleep(100);
        moveX(300);
        sleep(100);
        wobbleGrip.setPosition(Constants.lowerWobbleUp);
        sleep(1000);
        moveY(Constants.wobbleY+900);
        sleep(100);
        //moveY(Constants.CLowerBorder);
        //sleep(100);
        moveX(Constants.wobbleX+80);
        sleep(100);
        turnTo(180);
        yDirect = -1;
        xDirect = -1;
        sleep(100);
//	        moveY(Constants.wobbleY+600);
//            sleep(100);
        //moveX(Constants.wobbleX);
        moveY(Constants.wobbleY+70);
        sleep(100);
        turnTo(180);
        sleep(100);
        wobbleGrip.setPosition(lowerWobbleDown);
        sleep(100);
        moveX(100);
        sleep(100);
        turnTo(0);
        xDirect =1;
        yDirect = 1;
        sleep(100);
        moveY(Constants.CLowerBorder);
        sleep(100);
        moveY(Constants.navLineY);
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
        sleep(100);
    }

}
//Path A:
	        /*
            wobbleGrip.setPosition(Constants.lowerWobbleDown);
            sleep(1000);
            moveY(Constants.ALowerBorder);
            sleep(100);
	        moveX(-400);
            wobbleGrip.setPosition(Constants.lowerWobbleUp);
            moveY(Constants.ALowerBorder-200);
            //turnTo(0);
            sleep(100);
	        moveX(655);
            turnTo(180);
            yDirect = -1;
            xDirect = -1;
            sleep(100);

            moveY(Constants.wobbleY+42);
            sleep(1000);
            wobbleGrip.setPosition(Constants.lowerWobbleDown);

	         */
//            sleep(100);
//            moveX(-10);
//            moveY(Constants.ALowerBorder);
//            wobbleGrip.setPosition(Constants.lowerWobbleUp);
//            sleep(100);;
//            moveY(Constants.navLineY);
