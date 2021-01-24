package org.firstinspires.ftc.teamcode.java.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.java.util.*;

/**
 * the AutoAdjusting class will be used to adjust the angle in two axis of the shooter
 */
public class AutoAdjusting{


    RobotHardware robot;
    private final AnalogInput potentiometer;
    public AutoAdjusting(RobotHardware robot){
        this.robot =robot;
        potentiometer = robot.potentiometer;

    }

    /**
     * adjusting the pitch angle (using PIDF)
     */
    public void adjustPitch(){

    }

    /**
     * adjusting the yaw angle (using PID)
     */

    public void adjustYaw (){
    }

    public double getShooterPitchAngle() {
        return (potentiometer.getVoltage()*81.8);
    }
}
