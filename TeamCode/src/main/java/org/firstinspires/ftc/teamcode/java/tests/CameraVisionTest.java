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

		RingHeightPipeline pipeline = new RingHeightPipeline(telemetry);

	    OpenCvInternalCamera camera;
	    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
			    "cameraMonitorViewId",
			    "id",
			    hardwareMap.appContext.getPackageName()
	    );
	    camera = OpenCvCameraFactory.getInstance().createInternalCamera(
			    OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId
	    );

	    camera.setPipeline(pipeline);

	    camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

	    camera.openCameraDeviceAsync(() ->
			    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
	    );

	    waitForStart();

	    while (opModeIsActive()) {
            telemetry.addData("Height Position", pipeline.getHeight());
//            if (pipeline.getHeight()== RingHeightPipeline.Height.C){
//            	telemetry.speak("Yeee");
//            	sleep(3000);
//            	requestOpModeStop();

            //}
            telemetry.update();
	    }

	    camera.stopStreaming();
//
//        HeightDetector heightDetector = new HeightDetector(hardwareMap);
//        waitForStart();
//
//        while (opModeIsActive()) {
//            telemetry.addData("Height Position", heightDetector.getHeight());
//            telemetry.update();
//            sleep(50);
//        }
    }
}
