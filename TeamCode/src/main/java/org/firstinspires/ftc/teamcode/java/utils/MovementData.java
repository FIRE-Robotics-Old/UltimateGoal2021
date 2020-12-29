package org.firstinspires.ftc.teamcode.java.utils;

public final class MovementData {
    private final Coordinate coordinate;
    private final double angle;

    public MovementData(double x, double y, double angle) {
        this.coordinate = new Coordinate(x, y);
        this.angle = angle;
    }

    public MovementData(Coordinate coordinate, double angle) {
        this.coordinate = coordinate;
        this.angle = angle;
    }

    public double getAngle() {
        return this.angle;
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

        if (Double.compare(that.getAngle(), getAngle()) != 0) return false;
        return coordinate.equals(that.coordinate);
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = coordinate.hashCode();
        temp = Double.doubleToLongBits(getAngle());
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}
