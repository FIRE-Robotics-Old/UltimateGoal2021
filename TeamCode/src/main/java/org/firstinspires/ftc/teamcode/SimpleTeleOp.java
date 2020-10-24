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

           boolean up              = false;
           boolean right           = false;
           boolean down            = false;
           boolean left            = false;

           double  speed           = 0.3;
           double  still           = 0;

           int     currentButton   = 4; //none

    @Override
             public void runOpMode() throws InterruptedException {
        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor" );
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor"  );
        backRightMotor  = hardwareMap.dcMotor.get("backRightMotor" );

        frontLeftMotor.setDirection (DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection (DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        //Set Zero Power Behavior To All Motors

        frontLeftMotor.setPower (still);
        frontRightMotor.setPower(still);
        backLeftMotor.setPower (still);
        backRightMotor.setPower(still);


        waitForStart();

        while (opModeIsActive()) {
            up              = gamepad1.dpad_up;
            right           = gamepad1.dpad_right;
            down            = gamepad1.dpad_down;
            left            = gamepad1.dpad_left;

            directionPriority(); //updates the current button

            if (currentButton == 0) {
                frontLeftMotor.setPower (speed);
                frontRightMotor.setPower(speed);
                backLeftMotor.setPower (speed);
                backRightMotor.setPower(speed);
            } else if (currentButton == 1) {
                frontLeftMotor.setPower (-1*speed);
                frontRightMotor.setPower(speed);
                backLeftMotor.setPower (speed);
                backRightMotor.setPower(-1*speed);
            } else if (currentButton == 2) {
                frontLeftMotor.setPower (-1*speed);
                frontRightMotor.setPower(-1*speed);
                backLeftMotor.setPower (-1*speed);
                backRightMotor.setPower(-1*speed);
            } else if (currentButton == 3) {
                frontLeftMotor.setPower (speed);
                frontRightMotor.setPower(-1*speed);
                backLeftMotor.setPower (-1*speed);
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
    //Priorities latest button pressed in favor of left or in favor of up?

    public void directionPriority() {

        boolean[] status = {up, right, down, left};
        for (int num = 0; num < 4; num++) {
            //If currentButton does not = num and status index num is true reassign
            if (currentButton != num && status[num] == true) {
                currentButton = num;
            }


        }
        if (status[0] == false && status[1] == false && status[2] == false && status[3] == false){
            currentButton = 4;
        }


    }

}

