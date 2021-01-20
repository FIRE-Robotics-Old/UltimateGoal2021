package org.firstinspires.ftc.teamcode.java.util;

import java.util.Locale;

public class Angle {
    private static final double TAU = 2 * Math.PI;

    private final double angle;

    private Angle(double angle) {
        this.angle = -angle;
    }

    public static Angle fromDegrees(double angle) {
        return new Angle(Math.toRadians(angle % 360));
    }

    public static Angle fromRadians(double angle) {
        return new Angle(angle % 360);
    }

    public double getAngleInRadians() {
        return angle;
    }

    public double getAngleInDegrees() {
        return Math.toDegrees(angle);
    }

    public double getTrimmedAngleInRadians() {
        if (angle > Math.PI) {
            return angle - TAU;
        } else if (angle <= - Math.PI) {
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
