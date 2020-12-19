package org.firstinspires.ftc.teamcode.java.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.java.cameravision.HeightDetector;

@TeleOp(name = "Camera Vision Test", group = "Testing")
public class CameraVisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HeightDetector heightDetector = new HeightDetector(hardwareMap);
        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Height Position", heightDetector.getHeight());
            telemetry.update();
            sleep(50);
        }
    }
}
