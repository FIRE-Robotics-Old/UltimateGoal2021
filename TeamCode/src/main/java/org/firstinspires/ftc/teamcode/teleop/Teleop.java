package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.models.Hardware;

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

    double drive = 0;
    double strafe = 0;
    double twist = 0;
    double intakeAndDelivery = 0;
    boolean ifReversedIntakePressed = false;
    boolean shooterIsPressed = true;
    @Override
    public void runOpMode() {
        fL = robot.frontLeftMotor;
        fR = robot.frontRightMotor;
        bL = robot.backLeftMotor;
        bL = robot.backRightMotor;



        waitForStart();


        while (opModeIsActive()) {
            drive = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            twist = gamepad1.right_stick_x;
            intakeAndDelivery = gamepad2.left_trigger;

            if (gamepad2.back && ifReversedIntakePressed == false)
            {
                intakeAndDelivery *=-1;
                ifReversedIntakePressed =  true;
            }
            else
            {
                ifReversedIntakePressed = false;
            }

            double [] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            double max = Math.abs((speeds [0])*2);
            for (double speed : speeds) {
                if (max < Math.abs(speed)) max = Math.abs(speed);
            }

            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;

            }

            fL.setPower(speeds[0]);
            fR.setPower(speeds[1]);
            bL.setPower(speeds[2]);
            bR.setPower(speeds[3]);
            iD.setPower(intakeAndDelivery);



            if (gamepad2.a == true && shooterIsPressed == true)
            {
                lS.setPower(1);
                rS.setPower(1);
            }
            else {
                lS.setPower(0);
                rS.setPower(0);
            }


        }

    }
}
