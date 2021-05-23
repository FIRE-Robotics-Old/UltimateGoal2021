package org.firstinspires.ftc.teamcode.java.util;

import org.firstinspires.ftc.teamcode.java.util.PositionControl.PidfController;

public final class PidfConstants {
	//TODO get rid of this in library
		//US

	//decent values
//	static double max = 0.00065444437449;
//	static double min = 0.000654249921305;
	//public static double average = (max + min)/2;
	// Ku 0.00258
	// Kp  = 0.001548
	// Ki = 0.00238153846
	// Kd = 0.00024375
	public static double average = 0.0014654455;
	// previous value: 0.00765445
	// previous value: 0.000765445
	//nice value = 0.00065445 ⇒ Good Value for P
	/*
	*
	* */
	// https://www.youtube.com/watch?v=nvAQHSe-Ax4
	public static final PidfController USDrive = new PidfController(0, 0, 0, 0);
	//1st 0.000643555static double max = 0.00065444437449;
	//	static double min = 0.000654249921305;
	//2nd 0.0006497775
	//3rd 0.000652888749
	//4th 0.00065444437449
	//5th 0.000653666561745
	//6th 0.00065405546812
	//7th 0.000654249921305
	//public static final PidfController USDrive = new PidfController(average, 0, 0.0/*2605/*125*/, 0);
	//public static final PidfController USDrive = new PidfController( 0.00065457, 0, 0.003, 0);

	//decent low 0.0005564375
	//decent high 0.000656
	//start with .0005564375
	//public static final PidfController USStrafe = new PidfController(0, 0, 0, 0);
	public static final PidfController USStrafe = new PidfController(0.0003, 0, 0, 0);
	public static final PidfController USTurn = new PidfController(0, 0, 0, 0);
	//public static final PidfController USTurn = new PidfController(0.13255, 0, 0./*3695*/, 0); //.35
	//turn decent low: 0.1205s
	//turn decent high: 0.1325
		//Israel
	public static final PidfController ISDrive = new PidfController(0,0,0,0);
	public static final PidfController ISStrafe = new PidfController(0,0,0,0);
	public static final PidfController ISTurn = new PidfController(0,0,0,0);
}
