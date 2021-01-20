package org.firstinspires.ftc.teamcode.java.util;

public enum Units {
    NANOMETER   (Constants.nanometerToMillimeter),
    MICROMETER  (Constants.micrometerToMillimeter),
    MILLIMETERS (Constants.millimeterToMillimeter),
    CENTIMETERS (Constants.centimeterToMillimeter),
    DECIMETERS  (Constants.decimeterToMillimeter),
    METERS      (Constants.meterToMillimeter),
    DEKAMETERS  (Constants.dekameterToMillimeter),
    HECTOMETER  (Constants.hectometerToMillimeter),
    KILOMETERS  (Constants.kilometerToMillimeter),
    INCHES      (Constants.inchToMillimeter),
    FEET        (Constants.footToMillimeter),
    YARDS       (Constants.yardToMillimeter),
    MILES       (Constants.mileToMillimeter);

    private final double multiplier;

    Units(double multiplier) {
        this.multiplier = multiplier;
    }

    public double getMultiplier() {
        return this.multiplier;
    }
}