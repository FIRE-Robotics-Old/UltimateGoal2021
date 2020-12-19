package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
(name="Basic Omnidirectional TeleOp", group="Linear Opmode")
public class BasicOmnidirectionalTeleOp extends LinearOpMode {

    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;

    @Override
    public void runOpMode() {
        fL = hardwareMap.dcMotor.get("frontLeftMotor");
        fR = hardwareMap.dcMotor.get("frontRightMotor");
        bL = hardwareMap.dcMotor.get("backLeftMotor");
        bR = hardwareMap.dcMotor.get("backRightMotor");

        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

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

        }

    }
}
