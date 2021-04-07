package org.firstinspires.ftc.teamcode.java.util;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.java.util.Constants.PI;
import static org.firstinspires.ftc.teamcode.java.util.Constants.TAU;

public class Angle {
	private final double angle;

	private Angle(double angle) {
		this.angle = angle;
	}

	public Angle makePositive() {
		return new Angle(angle < 0 ? -angle : angle);
	}

	public static Angle fromDegrees(double angle, boolean reflectDirection) {
		return fromRadians(Math.toRadians(angle), reflectDirection);
	}

	public static Angle fromDegrees(double angle) {
		return fromDegrees(angle, true);
	}

	public static Angle fromRadians(double angle, boolean reflectDirection) {
		// If Angle < 0, We want to maintain the sign but fix the modulus, which means that we reflect
		// it again, use the modulus, and undo the reflection. Otherwise, simply apply the modulus operation.
		return new Angle((reflectDirection ? -1 : 1) * (angle < 0 ? -((-angle) % TAU) : (angle % TAU)));
	}

	public static Angle fromRadians(double angle) {
		return fromRadians(angle, true);
	}

	public double getAngleInRadians() {
		return angle;
	}

	public double getAngleInDegrees() {
		return Math.toDegrees(angle);
	}

	public double getTrimmedAngleInRadians() {
		if (angle > PI) {
			return angle - TAU;
		} else if (angle < -PI) {
			return angle + TAU;
		} else return angle;
	}

	public double getTrimmedAngleInDegrees() {
		return Math.toDegrees(getTrimmedAngleInRadians());
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;

		Angle angle1 = (Angle) o;

		return Double.compare(angle1.angle, angle) == 0;
	}

	@Override
	public int hashCode() {
		long temp = Double.doubleToLongBits(angle);
		return (int) (temp ^ (temp >>> 32));
	}

	@Override
	public String toString() {
		return String.format(Locale.ENGLISH, "%.3f °", getAngleInDegrees());
	}

	public String toStringRadians() {
		return String.format(Locale.ENGLISH, "%.3f rad", getAngleInRadians());
	}
}
