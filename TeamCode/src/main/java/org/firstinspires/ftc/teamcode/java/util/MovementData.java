package org.firstinspires.ftc.teamcode.java.util;

import java.util.Locale;

public final class MovementData {
    private final Vector2D vector2D;
    private final double angle;

    /**
     * @param x the x coordinate
     * @param y the y coordinate
     * @param angle the angle
     * @deprecated
     */
    public MovementData(double x, double y, double angle) {
        this.vector2D = new Vector2D(x, y);
        this.angle = angle;
    }

    /**
     * @param vector2D the coordinate to move to
     * @param angle the angle
     * @deprecated
     */
    public MovementData(Vector2D vector2D, double angle) {
        this.vector2D = vector2D;
        this.angle = angle;
    }

    private MovementData(Vector2D vector2D, double angle, boolean inDegrees) {
        this.vector2D = vector2D;
        this.angle = (inDegrees ? Math.toRadians(angle) : angle);
    }

    private MovementData(double x, double y, double angle, boolean inDegrees) {
        this(new Vector2D(x, y), angle, inDegrees);
    }

    public static MovementData withDegrees(double x, double y, double angle) {
        return new MovementData(x, y, angle, true);
    }

    public static MovementData withDegrees(Vector2D vector2D, double angle) {
        return new MovementData(vector2D, angle, true);
    }

    public static MovementData withRadians(double x, double y, double angle) {
        return new MovementData(x, y, angle, false);
    }

    public static MovementData withRadians(Vector2D vector2D, double angle) {
        return new MovementData(vector2D, angle, false);
    }

    public double getAngleInRadians() {
        return ((angle + (2 * Math.PI)) % (2 * Math.PI));
    }

    public double getAngleInDegrees() {
        return ((Math.toDegrees(angle) + 360) % 360);
    }

    public double getX() {
        return vector2D.getX();
    }

    public double getY() {
        return vector2D.getY();
    }

    public Vector2D Coordinate() {
        return vector2D;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MovementData that = (MovementData) o;

        if (Double.compare(that.getAngleInRadians(), getAngleInRadians()) != 0) return false;
        return vector2D.equals(that.vector2D);
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = vector2D.hashCode();
        temp = Double.doubleToLongBits(getAngleInRadians());
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%s at %.2f Degrees", vector2D, getAngleInDegrees());
    }
}
