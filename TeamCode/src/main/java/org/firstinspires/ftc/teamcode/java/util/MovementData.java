package org.firstinspires.ftc.teamcode.java.util;

import java.util.Locale;

public final class MovementData {
    private final Vector2d translationalMovement;
    private final Angle angle;

    public MovementData(Vector2d translationalMovement, Angle angle) {
        this.translationalMovement = translationalMovement;
        this.angle = angle;
    }

    public MovementData(double x, double y, Angle angle) {
        this(new Vector2d(x, y), angle);
    }

    /**
     * @deprecated Use Constructor with {@link Angle}, or {@link MovementData(Vector2d, Angle)}
     */
    @Deprecated
    private MovementData(Vector2d translationalMovement, double angle, boolean inDegrees) {
        this(
            translationalMovement,
            inDegrees
                ? Angle.fromDegrees(angle)
                : Angle.fromRadians(angle)
        );
    }

    /**
     * @deprecated Use Constructor with {@link Angle}, or {@link MovementData(double, double, Angle)}
     */
    @Deprecated
    private MovementData(double x, double y, double angle, boolean inDegrees) {
        this(new Vector2d(x, y), angle, inDegrees);
    }

    /**
     * @deprecated Use Constructor with {@link Angle}, or {@link MovementData(double, double, Angle)}
     */
    @Deprecated
    public static MovementData withDegrees(double x, double y, double angle) {
        return new MovementData(x, y, angle, true);
    }

    /**
     * @deprecated Use Constructor with {@link Angle}, or {@link MovementData(Vector2d, Angle)}
     */
    @Deprecated
    public static MovementData withDegrees(Vector2d translation, double angle) {
        return new MovementData(translation, angle, true);
    }

    /**
     * @deprecated Use Constructor with {@link Angle}, or {@link MovementData(double, double, Angle)}
     */
    @Deprecated
    public static MovementData withRadians(double x, double y, double angle) {
        return new MovementData(x, y, angle, false);
    }

    /**
     * @deprecated Use Constructor with {@link Angle}, or {@link MovementData(Vector2d, Angle)}
     */
    @Deprecated
    public static MovementData withRadians(Vector2d translation, double angle) {
        return new MovementData(translation, angle, false);
    }

    public double getAngleInRadians() {
        return angle.getAngleInRadians();
    }

    public double getAngleInDegrees() {
        return angle.getAngleInDegrees();
    }

    public Angle getAngle() {
        return angle;
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
