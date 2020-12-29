package org.firstinspires.ftc.teamcode.java.utils;

import java.util.Locale;
import java.util.Objects;

public final class Coordinate {
    private final double x;
    private final double y;

    public Coordinate(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Coordinate(Coordinate coordinate) {
        this.x = coordinate.getX();
        this.y = coordinate.getY();
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public MovementData withAngle(double angle) {
        return new MovementData(this, angle);
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "(%f, %f)", x, y);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Coordinate coordinate = (Coordinate) o;
        return Double.compare(coordinate.getX(), x) == 0 &&
                Double.compare(coordinate.getY(), y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getX(), getY());
    }
}
