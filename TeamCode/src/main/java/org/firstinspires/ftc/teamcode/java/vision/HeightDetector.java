package org.firstinspires.ftc.teamcode.java.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class HeightDetector {
	/**
	 * This connects to the phone's camera to capture an image of the rings.
	 * <p>
	 * Currently, this uses the {@link OpenCvInternalCamera} class to temporally get
	 * access to advanced features. However, when we switch to a web camera, this will have to be
	 * refactored to use the webcam.
	 */
	private OpenCvInternalCamera camera;
	private RingHeightPipeline pipeline;

	/**
	 * A Simple constructor which creates a Ring Height Detector Instance
	 * @param hardwareMap The {@link HardwareMap} Generated by OpMode to connect to the phone
	 */
	public HeightDetector(HardwareMap hardwareMap, Telemetry telemetry) {
		init(hardwareMap, telemetry);
	}

	/**
	 * Initializes the Object and starts streaming (will be split up later)
	 * <p>
	 * This connects the camera to the pipeline and initializes development.
	 * @param hardwareMap A HardwareMap which allows the creation of a phone instance
	 */
	private void init(HardwareMap hardwareMap, Telemetry telemetry) {
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"cameraMonitorViewId",
				"id",
				hardwareMap.appContext.getPackageName()
		);
		camera = OpenCvCameraFactory.getInstance().createInternalCamera(
				OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId
		);

		pipeline = new RingHeightPipeline(telemetry);

		// Sets up the Camera to use the pipeline which detects the height of the rings
		camera.setPipeline(pipeline);

		camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

		telemetry.addData("It has been init", "Ayo");
		telemetry.update();
	}

    public void startStreaming() {
        // Starts Streaming the Camera Contents to the phone
        camera.openCameraDeviceAsync(() ->
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );
    }

	public void stopStreaming() {
		camera.stopStreaming();
	}

    public RingHeightPipeline.Height getHeight() {
        return pipeline.getHeight();
    }
}
