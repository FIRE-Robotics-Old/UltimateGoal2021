package org.firstinspires.ftc.teamcode.java.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.java.vision.HeightDetector;
import org.firstinspires.ftc.teamcode.java.vision.RingHeightPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "Camera Vision Test", group = "Testing")
public class CameraVisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        while (opModeIsActive()) {
            telemetry.addData("Height Position", heightDetector.getHeight());
            telemetry.update();
            sleep(50);
        }
    }
}
