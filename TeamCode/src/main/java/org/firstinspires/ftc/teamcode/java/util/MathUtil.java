package org.firstinspires.ftc.teamcode.java.util;

public class MathUtil {
	public static double squared(double num) {
		return num * num;
	}

	public static float squared(float num) {
		return num * num;
	}

	public static int squared(byte num) {
		return num * num;
	}

	public static int squared(short num) {
		return num * num;
	}

	public static long squared(int num) {
		return num * num;
	}

	public static long squared(long num) {
		return num * num;
	}

	public static double cube(double num) {
		return num * num * num;
	}

	public static float cube(float num) {
		return num * num * num;
	}

	public static int cube(byte num) {
		return num * num * num;
	}

	public static int cube(short num) {
		return num * num * num;
	}

	public static long cube(int num) {
		return num * num * num;
	}

	public static long cube(long num) {
		return num * num * num;
	}

	public static double pow(double num, int times) {
		while (--times != 0) num *= num;
		return num;
	}

	public static float pow(float num, int times) {
		while (--times != 0) num *= num;
		return num;
	}

	public static int pow(byte num, int times) {
		while (--times != 0) num *= num;
		return num;
	}

	public static int pow(short num, int times) {
		while (--times != 0) num *= num;
		return num;
	}

	public static long pow(int num, int times) {
		while (--times != 0) num *= num;
		return num;
	}

	public static long pow(long num, long times) {
		while (--times != 0) num *= num;
		return num;
	}
}
