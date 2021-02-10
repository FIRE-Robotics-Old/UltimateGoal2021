package org.firstinspires.ftc.teamcode.java.util;

public final class Constants {
	public static final double PI         = Math.PI;
	public static final double TAU        = 2 * PI;
	public static final double PI_OVER_2  = PI / 2;
	public static final double PI_OVER_4  = PI / 4;

	public static final double SIN_60 = Math.sqrt(3) / 2;
	public static final double COS_30 = Math.sqrt(3) / 2;
	public static final double SIN_45 = Math.sqrt(2) / 2;
	public static final double COS_45 = Math.sqrt(2) / 2;
	public static final double SIN_30 = 0.5;
	public static final double COS_60 = 0.5;

	public static final double E   = Math.E;
	public static final double E_1 = E;
	public static final double E_2 = E * E;
	public static final double E_3 = E * E * E;
	public static final double E_4 = E * E * E * E;
	public static final double E_5 = E * E * E * E * E;

	public static final double nanometerToMillimeter  = 10E-6;
	public static final double micrometerToMillimeter = 10E-3;
	public static final double millimeterToMillimeter = 10E00;
	public static final double centimeterToMillimeter = 10E01;
	public static final double decimeterToMillimeter  = 10E02;
	public static final double meterToMillimeter      = 10E03;
	public static final double dekameterToMillimeter  = 10E04;
	public static final double hectometerToMillimeter = 10E05;
	public static final double kilometerToMillimeter  = 10E06;

	public static final double inchToMillimeter = 25.4;
	public static final double footToMillimeter = 304.8;
	public static final double yardToMillimeter = 914.4;
	public static final double mileToMillimeter = 4828032;

	// Robotics Specific Constants
	public static final double robotLength = 444.5; //∆
	public static final double robotWidth = 457.2; //jajaja
		//field constants
	public static final double backFieldY = 3587.75;
	public static final double navLineY = 2032;
	public static final Coordinate cornerAB = new Coordinate(1828.8,2438.4);
	public static final Coordinate cornerBC = new Coordinate(1828.8,3048);

		//Hardware constants
	public static final double lowerWobbleDown = 0;
	public static final double lowerWobbleUp = 0.7;


}
