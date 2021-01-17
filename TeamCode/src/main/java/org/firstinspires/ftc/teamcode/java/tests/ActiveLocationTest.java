package org.firstinspires.ftc.teamcode.java.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.movement.AutoDriving;
import org.firstinspires.ftc.teamcode.java.movement.PathFinder;
import org.firstinspires.ftc.teamcode.java.util.PIDFController;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

//To fix error perhaps flip the switch

@Autonomous(name="ActiveLocationTest", group="Backup")

public class ActiveLocationTest extends LinearOpMode {

    // Declare OpMode members.
    RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    //Hardware robot = new Hardware();
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private BNO055IMU imu;
    private ActiveLocation AL;
    private Thread locationThread;
    private PathFinder PF;
    private Thread pathThread;
    private AutoDriving autoDriving;
    private PIDFController PIDF;

    //private ElapsedTime runtime = new ElapsedTime();
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

        PF = new PathFinder(AL);
        pathThread = new Thread(PF);
        pathThread.start();

        PIDF = new PIDFController(.75, 0.007, 0.25, 0);
        //autoDriving = new AutoDriving(PIDF, robot);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        try{
            AL.setStartPosition(0, 0, 0);
            PF.setDestination(600, 600, 90);
            /*
            telemetry.addData("FL", frontLeftMotor.getCurrentPosition());
            telemetry.addData("BR", backRightMotor.getCurrentPosition());
            telemetry.addData("FR", frontRightMotor.getCurrentPosition());
            telemetry.addData("BL", backLeftMotor.getCurrentPosition());
            telemetry.update();
        }*/
            while (opModeIsActive()) {
                //telemetry.addData("X", AL.getFieldX());
                //telemetry.addData("Y", AL.getFieldY());
                telemetry.addData("Angle", Math.toDegrees(imu.getAngularOrientation().firstAngle));
                telemetry.addData("Pain", AL.getAngleInDegrees());
                telemetry.addData("Path: ", PF.getEncoderPath());
                //telemetry.addData("Error",autoDriving.errorReport(MovementData.withDegrees(600,600,90)));
                telemetry.update();
                sleep(300);
            }

            AL.stop();
            PF.stop();
        }catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            //AL.Stop();
            //PF.stop();
            telemetry.update();
        }
    }
}
