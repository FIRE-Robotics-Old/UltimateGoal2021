package org.firstinspires.ftc.teamcode.java.utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.Collections;

/**
 * the PIDController class will be used for  all the different pid calculations
 * for example AutoDriving ,autoAdjusting , controlling the shooter speed
 */
//P is main power, I looks at the sum of error and gives final push, D is how much the error is changing
public class PIDFController {
    private final ElapsedTime elapsedTime;
    private final double previousTime;
    private final double ki;
    private final double kp;
    private final double kd;
    private final double f;
    private final double maxI = 1;
    private final double minI = -maxI;
    private double xi = 0;
    private double yi = 0;
    private double ai = 0;
    private double xd = 0;
    private double yd = 0;
    private double ad = 0;

    public PIDFController(double previousTime, double kp, double ki, double kd, double f) {
        elapsedTime = new ElapsedTime();
        this.previousTime = elapsedTime.time();
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.f = f;

    }

    //TODO create object for x,y,angle and x,y
    public Double[] calculateDrivePowers(double maxV, Coordinate error, double angleError) {
        double xError = error.getX();
        double yError = error.getY();
        double strafe = calculateDrivePID(xError, Which.Strafe);
        double drive = calculateDrivePID(yError, Which.Drive);
        double twist = calculateDrivePID(angleError, Which.Twist);
        Double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        //finding the biggest drive motor power
        double max = Collections.max(Arrays.asList(speeds));


        //setting the max speed while keeping the ratio
        // change the number to change the max speed (0-1)


        if (max > maxV) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= (1 / maxV) * max;
        }
        return speeds;

    }

    private double calculateDrivePID(double error, Which which) {
        double currentTime = elapsedTime.time();
        double p = kp * error;
        double i;
        switch (which) {
            case Strafe:
                i = Range.clip(xi + ki * (error * (currentTime - previousTime)), minI, maxI);
                xi = i;
                break;
            case Drive:
                i = Range.clip(yi + ki * (error * (currentTime - previousTime)), minI, maxI);
                yi = i;
                break;
            default:
                i = Range.clip(ai + ki * (error * (currentTime - previousTime)), minI, maxI);
                ai = i;
        }

        double d;
        switch (which) {
            case Strafe:
                d = kd * ((error - xd) / (currentTime - previousTime));
                xd = error;
                break;
            case Drive:
                d = kd * ((error - yd) / (currentTime - previousTime));
                yd = error;
                break;
            default:
                d = kd * ((error - ad) / (currentTime - previousTime));
                ad = error;
        }
        double output = p + i + d;
        return f + (1 - f) * output; //Scales

    }

    enum Which {
        Drive,
        Strafe,
        Twist
    }


}
