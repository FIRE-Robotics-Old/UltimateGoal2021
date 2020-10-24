package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Simple TeleOp", group="Basic TeleOp")
public class SimpleTeleOp extends LinearOpMode {

    public DcMotor frontRightMotor;
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

           boolean up              = gamepad1.dpad_up;
           boolean right           = gamepad1.dpad_right;
           boolean down            = gamepad1.dpad_down;
           boolean left            = gamepad1.dpad_left;

           double  speed           = 0.3;
           double  still           = 0;

           int     currentButton   = 0;

    @Override
             public void runOpMode() throws InterruptedException {
        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor" );
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor"  );
        backRightMotor  = hardwareMap.dcMotor.get("backRightMotor" );

        frontLeftMotor.setDirection (DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection (DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);


        //Set Zero Power Behavior To All Motors

        frontLeftMotor.setPower (still);
        frontRightMotor.setPower(still);
        backLeftMotor.setPower (still);
        backRightMotor.setPower(still);


        waitForStart();

        while (opModeIsActive()) {

            if (up) {
                frontLeftMotor.setPower (speed);
                frontRightMotor.setPower(speed);
                backLeftMotor.setPower (speed);
                backRightMotor.setPower(speed);
            } else if (right) {
                frontLeftMotor.setPower (speed);
                frontRightMotor.setPower(speed);
                backLeftMotor.setPower (speed);
                backRightMotor.setPower(speed);
            } else if (down) {
                frontLeftMotor.setPower (speed);
                frontRightMotor.setPower(speed);
                backLeftMotor.setPower (speed);
                backRightMotor.setPower(speed);
            } else if (left) {
                frontLeftMotor.setPower (speed);
                frontRightMotor.setPower(speed);
                backLeftMotor.setPower (speed);
                backRightMotor.setPower(speed);
            } else {
                //Set Zero Power Behavior To All Motors
                frontLeftMotor.setPower (still);
                frontRightMotor.setPower(still);
                backLeftMotor.setPower (still);
                backRightMotor.setPower(still);

            }


        }




    }


            /*

            double [] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            double max = Math.abs(speeds [0]);
            for (int i = 0; i < speeds.length; i++) {
                if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
            }

            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;

            }

            fL.setPower(speeds[0]);
            fR.setPower(speeds[1]);
            bL.setPower(speeds[2]);
            bR.setPower(speeds[3]);
/*
        }

    }
}

             */

    public int directionPriority() {

        boolean[] status = {up, right, down, left};
        for(int num = 0; num < 3; num ++) {

        }
        return 0;
    }

}

