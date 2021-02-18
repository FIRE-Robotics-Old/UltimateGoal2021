package org.firstinspires.ftc.teamcode.java.util;

import java.util.Locale;

public class Vector3d {
	private final double x;
	private final double y;
	private final double z;

	public Vector3d(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Vector3d(Vector3d vector3D) {
		this.x = vector3D.getX();
		this.y = vector3D.getY();
		this.z = vector3D.getZ();
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getZ() {
		return z;
	}

	public double magnitude() {
		return Math.sqrt(x * x + y * y + z * z);
	}

	public double vectorLength() {
		return magnitude();
	}

	public Vector3d plus(Vector3d other){
		return new  Vector3d(x + other.x, y + other.y, z + other.z);
	}

	public Vector3d minus(Vector3d other){
		return plus(other.unaryMinus());
	}

	public Vector3d unaryMinus() {
		return new Vector3d(-x, -y, -z);
	}

	public Vector3d times(double scalar) {
		return new Vector3d(x * scalar, y * scalar, z * scalar);
	}

	public Vector3d div(double scalar) {
		return new Vector3d(x / scalar, y / scalar, z / scalar);
	}

	@Override
	public String toString() {
		return String.format(Locale.ENGLISH, "(%.2f, %.2f, %.2f)", x, y, z);
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;

		Vector3d vector3d = (Vector3d) o;

		if (Double.compare(vector3d.getX(), getX()) != 0) return false;
		if (Double.compare(vector3d.getY(), getY()) != 0) return false;
		return Double.compare(vector3d.getZ(), getZ()) == 0;
	}

	@Override
	public int hashCode() {
		int result;
		long temp;
		temp = Double.doubleToLongBits(getX());
		result = (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(getY());
		result = 31 * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(getZ());
		result = 31 * result + (int) (temp ^ (temp >>> 32));
		return result;
	}
}
