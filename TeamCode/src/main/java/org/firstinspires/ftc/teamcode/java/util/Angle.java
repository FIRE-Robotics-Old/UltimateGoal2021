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
		return new Angle((reflectDirection ? -1 : 1) * Math.toRadians(angle % 360));
	}

	public static Angle fromDegrees(double angle) {
		return fromDegrees(angle, true);
	}

	public static Angle fromRadians(double angle, boolean reflectDirection) {
		return new Angle((reflectDirection ? -1 : 1) * (angle % (TAU)));
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
		} else if (angle <= PI) {
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
