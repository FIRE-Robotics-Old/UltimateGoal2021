package org.firstinspires.ftc.teamcode.java.utils;

import java.util.Locale;

public final class MovementData {
    private final Coordinate coordinate;
    private final double angle;

    /**
     * @param x the x coordinate
     * @param y the y coordinate
     * @param angle the angle
     * @deprecated
     */
    public MovementData(double x, double y, double angle) {
        this.coordinate = new Coordinate(x, y);
        this.angle = angle;
    }

    /**
     * @param coordinate the coordinate to move to
     * @param angle the angle
     * @deprecated
     */
    public MovementData(Coordinate coordinate, double angle) {
        this.coordinate = coordinate;
        this.angle = angle;
    }

    private MovementData(Coordinate coordinate, double angle, boolean inDegrees) {
        this.coordinate = coordinate;
        this.angle = (inDegrees ? Math.toRadians(angle) : angle);
    }

    private MovementData(double x, double y, double angle, boolean inDegrees) {
        this(new Coordinate(x, y), angle, inDegrees);
    }

    public static MovementData withDegrees(double x, double y, double angle) {
        return new MovementData(x, y, angle, true);
    }

    public static MovementData withDegrees(Coordinate coordinate, double angle) {
        return new MovementData(coordinate, angle, true);
    }

    public static MovementData withRadians(double x, double y, double angle) {
        return new MovementData(x, y, angle, false);
    }

    public static MovementData withRadians(Coordinate coordinate, double angle) {
        return new MovementData(coordinate, angle, false);
    }

    public double getAngleInRadians() {
        return ((angle + (2 * Math.PI)) % (2 * Math.PI));
    }

    public double getAngleInDegrees() {
        return ((Math.toDegrees(angle) + 360) % 360);
    }

    public double getX() {
        return coordinate.getX();
    }

    public double getY() {
        return coordinate.getY();
    }

    public Coordinate Coordinate() {
        return coordinate;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MovementData that = (MovementData) o;

        if (Double.compare(that.getAngleInRadians(), getAngleInRadians()) != 0) return false;
        return coordinate.equals(that.coordinate);
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = coordinate.hashCode();
        temp = Double.doubleToLongBits(getAngleInRadians());
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%s at %.2f Degrees", coordinate, getAngleInDegrees());
    }
}
