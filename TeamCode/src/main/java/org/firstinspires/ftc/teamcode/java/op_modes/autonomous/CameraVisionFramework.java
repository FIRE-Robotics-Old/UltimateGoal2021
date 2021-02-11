package org.firstinspires.ftc.teamcode.java.op_modes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.java.vision.HeightDetector;
import org.firstinspires.ftc.teamcode.java.vision.RingHeightPipeline;

public class CameraVisionFramework extends LinearOpMode {
	@Override
	public void runOpMode() {
		// OpMode Necessities
		HeightDetector heightDetector = new HeightDetector(hardwareMap);

		waitForStart();

		RingHeightPipeline.Height position = heightDetector.getHeight();

		switch (position) {
			case A:
				// Do Whatever Zone A is
			case B:
				// Zone B
			case C:
				// Zone C
		}
	}
}
