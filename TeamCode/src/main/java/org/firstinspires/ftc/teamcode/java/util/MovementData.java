package org.firstinspires.ftc.teamcode.java.util;

import java.util.Locale;

public final class MovementData {
    private final Vector2d translationalMovement;
    private final double angle;

    /**
     * @param x the x coordinate
     * @param y the y coordinate
     * @param angle the angle
     * @deprecated
     */
    public MovementData(double x, double y, double angle) {
        this.translationalMovement = new Vector2d(x, y);
        this.angle = angle;
    }

    /**
     * @param translationalMovement the coordinate to move to
     * @param angle the angle
     * @deprecated
     */
    public MovementData(Vector2d translationalMovement, double angle) {
        this.translationalMovement = translationalMovement;
        this.angle = angle;
    }

    private MovementData(Vector2d translationalMovement, double angle, boolean inDegrees) {
        this.translationalMovement = translationalMovement;
        this.angle = (inDegrees ? Math.toRadians(angle) : angle);
    }

    private MovementData(double x, double y, double angle, boolean inDegrees) {
        this(new Vector2d(x, y), angle, inDegrees);
    }

    public static MovementData withDegrees(double x, double y, double angle) {
        return new MovementData(x, y, angle, true);
    }

    public static MovementData withDegrees(Vector2d translation, double angle) {
        return new MovementData(translation, angle, true);
    }

    public static MovementData withRadians(double x, double y, double angle) {
        return new MovementData(x, y, angle, false);
    }

    public static MovementData withRadians(Vector2d translation, double angle) {
        return new MovementData(translation, angle, false);
    }

    //TODO fix naming
    public double getAngleInRadians() {
        return ((angle + (2 * Math.PI)) % (2 * Math.PI));
    }

    public double getAngleInDegrees() {
        return ((Math.toDegrees(angle) + 360) % 360);
    }

    public double getRawAngleInRadians() {
        return (angle);
    }

    public double getRawAngleInDegrees() {
        return (Math.toDegrees(angle));
    }

    public double getX() {
        return translationalMovement.getX();
    }

    public double getY() {
        return translationalMovement.getY();
    }

    public Vector2d getTranslationalMovement() {
        return translationalMovement;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MovementData that = (MovementData) o;

        if (Double.compare(that.getAngleInRadians(), getAngleInRadians()) != 0) return false;
        return translationalMovement.equals(that.translationalMovement);
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = translationalMovement.hashCode();
        temp = Double.doubleToLongBits(getAngleInRadians());
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%s at %.2f Degrees", translationalMovement, getAngleInDegrees());
    }
}
