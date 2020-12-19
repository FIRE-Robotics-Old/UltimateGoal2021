package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.java.fieldmapping.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.utils.Hardware;




@TeleOp
        (name="teleOp", group="Linear Opmode")
public class Teleop extends LinearOpMode {
    Hardware robot;

    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;
    public DcMotor iD;
    public DcMotor lS;
    public DcMotor rS;

    private ActiveLocation AL;
    private Thread locationThread;

    double drive = 0;
    double strafe = 0;
    double twist = 0;
    double intakeAndDeliveryPower = 0;
    double shooterPower =0 ;
    boolean ifReversedIntakePressed = false;
    boolean shooterIsPressed = true;
    @Override
    public void runOpMode() {
        fL = robot.frontLeftMotor;
        fR = robot.frontRightMotor;
        bL = robot.backLeftMotor;
        bL = robot.backRightMotor;

        AL = new ActiveLocation(robot);
        locationThread = new Thread(AL);
        locationThread.start();


        waitForStart();
        //TODO mode change and angel reset  and shooter adjusting


        while (opModeIsActive()) {
            //motors powers calculation
            drive = gamepad1.left_stick_y *Math.cos(AL.getAngle()) -
                    gamepad1.left_stick_x * Math.sin(AL.getAngle()) ;
            strafe =gamepad1.left_stick_x *Math.cos(AL.getAngle()) +
                    gamepad1.left_stick_y * Math.sin(AL.getAngle()) ;
            twist = gamepad1.right_stick_x;
            intakeAndDeliveryPower = gamepad2.left_trigger;
            // wheel speed calculation
            double [] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };
            //finding the biggest drive motor power
            double max = Math.abs((speeds [0]));
            for (double speed : speeds) {
                if (max < Math.abs(speed)) max = Math.abs(speed);
            }
            //setting the max speed while keeping the ratio
            // change the number to change the max speed (0-1)
            // TODO change the max speed to 1
            if (max > 0.5) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;

            }

                // reversing the intake and delivery
            if (gamepad2.back && ifReversedIntakePressed == false)
            {
                intakeAndDeliveryPower *=-1;
                ifReversedIntakePressed =  true;
            }
            else
            {
                ifReversedIntakePressed = false;
            }

            //turning on the shooter

            if (gamepad2.a  && shooterIsPressed ==false)
            {
                shooterIsPressed = true;
                shooterPower=1;
            }
            else
            {
                shooterIsPressed=false;
                shooterPower=0;
            }
            //setting the speed to the motors
            fL.setPower(speeds[0]);
            fR.setPower(speeds[1]);
            bL.setPower(speeds[2]);
            bR.setPower(speeds[3]);
            iD.setPower(intakeAndDeliveryPower);
            lS.setPower(shooterPower);
            rS.setPower(shooterPower);


        }

    }
}
