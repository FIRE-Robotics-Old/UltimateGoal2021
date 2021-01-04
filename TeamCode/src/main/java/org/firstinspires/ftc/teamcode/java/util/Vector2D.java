package org.firstinspires.ftc.teamcode.java.util;

import java.util.Locale;
import java.util.Objects;

public final class Vector2D {
    private final double x;
    private final double y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D(Vector2D vector2D) {
        this.x = vector2D.getX();
        this.y = vector2D.getY();
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Vector2D rotateBy(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        double x = this.x * cos - this.y * sin;
        double y = this.x * sin + this.y * cos;

        return new Vector2D(x, y);
    }

    public Vector2D rotateByDegrees(double degrees) {
        return rotateBy(Math.toRadians(degrees));
    }

    public MovementData withAngleInRadians(double angle) {
        return MovementData.withRadians(this, angle);
    }

    public MovementData withAngleInDegrees(double angle) {
        return MovementData.withDegrees(this, angle);
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "(%.2f, %.2f)", x, y);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Vector2D vector2D = (Vector2D) o;
        return Double.compare(vector2D.getX(), x) == 0 &&
                Double.compare(vector2D.getY(), y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getX(), getY());
    }
}
