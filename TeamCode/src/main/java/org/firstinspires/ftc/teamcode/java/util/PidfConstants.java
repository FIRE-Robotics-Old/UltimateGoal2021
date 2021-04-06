package org.firstinspires.ftc.teamcode.java.util;

public final class PidfConstants {
	//TODO get rid of this in libary
		//US
	public static final PidfController USDrive = new PidfController(0, 0, 0, 0);
	//public static final PidfController USDrive = new PidfController(0.000562, 0, 0.0/*2605/*125*/, 0);
	public static final PidfController USStrafe = new PidfController(0, 0, 0, 0);
	//public static final PIDFController USStrafe = new PIDFController(0.00125, 0, 0, 0);
	//public static final PidfController USTurn = new PidfController(0, 0, 0, 0);
	public static final PidfController USTurn = new PidfController(0.1517, 0, 0./*3695*/, 0); //.35
	//turn decent: 0.27
		//Israel
	public static final PidfController ISDrive = new PidfController(0,0,0,0);
	public static final PidfController ISStrafe = new PidfController(0,0,0,0);
	public static final PidfController ISTurn = new PidfController(0,0,0,0);
}
