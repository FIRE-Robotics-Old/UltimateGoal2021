package org.firstinspires.ftc.teamcode.java.util;

public final class PIDFConstants {
	//TODO get rid sof this in libary
		//US
	public static final PIDFController USDrive = new PIDFController(0, 0, 0, 0);
	//public static final PIDFController USDrive = new PIDFController(0.00118, 0, 0.00125, 0);
	//public static final PIDFController USStrafe = new PIDFController(0.001705, 0.000000000000001, 0.005705, 0);
	public static final PIDFController USStrafe = new PIDFController(0.0017, 0, 0, 0);
	//public static final PIDFController USTurn = new PIDFController(0.35, 0.000000000, 0.395, 0);
	public static final PIDFController USTurn = new PIDFController(0, 0, 0, 0);
		//Israel
	public static final PIDFController ISDrive = new PIDFController(0,0,0,0);
	public static final PIDFController ISStrafe = new PIDFController(0,0,0,0);
	public static final PIDFController ISTurn = new PIDFController(0,0,0,0);
}
