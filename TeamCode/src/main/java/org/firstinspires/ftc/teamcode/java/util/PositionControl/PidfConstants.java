package org.firstinspires.ftc.teamcode.java.util.PositionControl;

public final class PidfConstants {
	//TODO get rid of this in library
		//US
	public static final PidfController USDrive = new PidfController(6.5057E-4, 0, 3E-3, 0);
	//public static final PIDFController USDrive = new PIDFController(0.00118, 0, 0.00125, 0);
	public static final PidfController USStrafe = new PidfController(0.0013, 0, 0.056, 0);
	//public static final PIDFController USStrafe = new PIDFController(0.00125, 0, 0, 0);
	//public static final PIDFController USTurn = new PIDFController(0, 0, 0, 0);
	public static final PidfController USTurn = new PidfController(0.1, 0, 0.18, 0);
		//Israel
	public static final PidfController ISDrive = new PidfController(0,0,0,0);
	public static final PidfController ISStrafe = new PidfController(0,0,0,0);
	public static final PidfController ISTurn = new PidfController(0,0,0,0);
}
