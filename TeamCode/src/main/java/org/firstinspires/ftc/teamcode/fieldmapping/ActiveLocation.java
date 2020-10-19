package org.firstinspires.ftc.teamcode.fieldmapping;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The RealTimeLocation uses odometry to find the real-time location of the robot. This,
 * along with a PathFinder, helps create a Field Mapping to allow us to accurately move to specific
 * positions in autonomous.
 */
public class ActiveLocation{ // add implements Runnable{
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    final static double tickPerRotation = 8192;
    final static double wheelCircumference = 90 * Math.PI;

    public ActiveLocation(DcMotor frontLeftMotor, DcMotor backRightMotor){
        this.frontLeftMotor = frontLeftMotor;
        this.backRightMotor = backRightMotor;
    }

    /**Figures out distance based on current encoder position over total ticks per rotation and muliplies it by circumfernce
      * @param ticks
     * @param encoder
     * @return
     */
    public static double tickToDistance(double ticks, DcMotor encoder){
        double currentTicks = encoder.getCurrentPosition();
        return ((currentTicks/tickPerRotation) * wheelCircumference);
    }

    public static double DistanceToTicks(double distance, DcMotor encoder){

        return ((distance/wheelCircumference)*tickPerRotation);
    }



}
