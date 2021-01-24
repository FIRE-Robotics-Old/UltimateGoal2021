package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
(name="Basic Omnidirectional TeleOp", group="Linear Opmode")
public class BasicOmnidirectionalTeleOp extends LinearOpMode {

    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;
    public DcMotor liftMotor;
    private Servo wobbleGrip;
    private Servo wobble2;
    private double open = .9;
    private double close = .5;
    public RevColorSensorV3 colorSensor;
    public int colors;


    @Override
    public void runOpMode() {
        fL = hardwareMap.dcMotor.get("frontLeftMotor");
        fR = hardwareMap.dcMotor.get("frontRightMotor");
        bL = hardwareMap.dcMotor.get("backLeftMotor");
        bR = hardwareMap.dcMotor.get("backRightMotor");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        wobbleGrip = hardwareMap.get(Servo.class, "wobbleGrip");
        wobble2 = hardwareMap.get(Servo.class, "wobble2");

        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
        colorSensor = hardwareMap.get(RevColorSensorV3.class,"colorSensor");


        waitForStart();

        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            double down = gamepad1.left_trigger;
            double up = gamepad1.right_trigger;
            colors = colorSensor.blue();
            telemetry.addData("Color:",colors);
            telemetry.update();

            double [] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive + strafe - twist),
                    (drive - strafe + twist)
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

            if (x){
                wobbleGrip.setPosition(.5);
                wobble2.setPosition(0);
                telemetry.speak("Open");
            }
            if (y){
                wobbleGrip.setPosition(0.9);
                wobble2.setPosition(1);
                telemetry.speak("close");
            }
            telemetry.update();
            if (up>.3 && down <.3){
                liftMotor.setPower(up);
            }
            else if (down>.3 && up <.3){
                liftMotor.setPower(-1*down);
            }else{
                liftMotor.setPower(0);
            }


        }

    }
}
