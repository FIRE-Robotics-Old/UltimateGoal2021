package org.firstinspires.ftc.teamcode.java.util;

public final class Distance {
    public static double fromNanometers(double nanometer) {
        return nanometer * Constants.nanometerToMillimeter;
    }

    public static double fromMicrometers(double micrometer) {
        return micrometer * Constants.micrometerToMillimeter;
    }

    public static double fromMillimeter(double millimeter) {
        return millimeter * Constants.micrometerToMillimeter;
    }

    public static double fromCentimeter(double centimeter) {
        return centimeter * Constants.centimeterToMillimeter;
    }

    public static double fromDecimeter(double decimeter) {
        return decimeter * Constants.decimeterToMillimeter;
    }

    public static double fromMeter(double meter) {
        return meter * Constants.meterToMillimeter;
    }

    public static double fromDekameter(double dekameter) {
        return dekameter * Constants.dekameterToMillimeter;
    }

    public static double fromHectometer(double hectometer) {
        return hectometer * Constants.hectometerToMillimeter;
    }

    public static double fromKilometer(double kilometer) {
        return kilometer * Constants.kilometerToMillimeter;
    }

    public static double fromInches(double inches) {
        return inches * Constants.inchToMillimeter;
    }

    public static double fromFeet(double feet) {
        return feet * Constants.footToMillimeter;
    }

    public static double fromYards(double yards) {
        return yards * Constants.yardToMillimeter;
    }

    public static double fromMiles(double miles) {
        return miles * Constants.mileToMillimeter;
    }

    public static double getDistance(double value, Units units) {
        return value * units.getMultiplier();
    }
}
