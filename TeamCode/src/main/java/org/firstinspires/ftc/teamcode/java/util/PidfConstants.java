package org.firstinspires.ftc.teamcode.java.util;

public final class PidfConstants {
	//TODO get rid of this in libary
		//US
	static double max = 0.000656;
	static double min = 0.00060622;
	public static double average = (max + min)/2;
	//public static final PidfController USDrive = new PidfController(0, 0, 0, 0);
	public static final PidfController USDrive = new PidfController(average, 0, 0.0/*2605/*125*/, 0);
	//decent low 0.0005564375
	//decent high 0.000656
	//start with .0005564375
	public static final PidfController USStrafe = new PidfController(0, 0, 0, 0);
	//public static final PIDFController USStrafe = new PIDFController(0.00125, 0, 0, 0);
	public static final PidfController USTurn = new PidfController(0, 0, 0, 0);
	//public static final PidfController USTurn = new PidfController(0.13255, 0, 0./*3695*/, 0); //.35
	//turn decent low: 0.1205s
	//turn decent high: 0.1325
		//Israel
	public static final PidfController ISDrive = new PidfController(0,0,0,0);
	public static final PidfController ISStrafe = new PidfController(0,0,0,0);
	public static final PidfController ISTurn = new PidfController(0,0,0,0);
}
