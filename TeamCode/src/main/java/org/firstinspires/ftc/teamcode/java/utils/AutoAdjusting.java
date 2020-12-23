package org.firstinspires.ftc.teamcode.java.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AutoAdjusting{


    Hardware robot;
    private AnalogInput potentiometer;
    public AutoAdjusting(Hardware robot){
        this.robot =robot;
        potentiometer = robot.potentiometer;

    }

    public double getShooterAngle() {
        return (potentiometer.getVoltage()*81.8);
    }
}
