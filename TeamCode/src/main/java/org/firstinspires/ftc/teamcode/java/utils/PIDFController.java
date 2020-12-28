package org.firstinspires.ftc.teamcode.java.utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.Collections;
import java.util.stream.Collectors;

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

    private static final double maxI = 1;
    private static final double minI = -maxI;

    private double strafeIntegral = 0;
    private double driveIntegral = 0;
    private double twistIntegral = 0;

    private double strafeDerivative = 0;
    private double driveDerivative = 0;
    private double twistDerivative = 0;

    public PIDFController(double kp, double ki, double kd, double f) {
        elapsedTime = new ElapsedTime();
        this.previousTime = 0;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.f = f;
    }

    //TODO create object for x,y,angle and x,y
    public double[] calculateDrivePowers(double maxV, Coordinate error, double angleError) {
        double xError = error.getX();
        double yError = error.getY();
        double strafe = calculateDrivePID(xError, Which.Strafe);
        double drive = calculateDrivePID(yError, Which.Drive);
        double twist = calculateDrivePID(angleError, Which.Twist);
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        // Finds the max after converting doubles to Doubles
        double max = Collections.max(Arrays.stream(speeds).boxed().collect(Collectors.toList()));

        if (max > maxV) {
            for (int i = 0; i < speeds.length; i++) speeds[i] *= maxV / max;
        }
        return speeds;
    }

    private double calculateDrivePID(double error, Which which) {
        double currentTime = elapsedTime.time();
        double p = kp * error;
        double i;
        switch (which) {
            case Strafe:
                i = Range.clip(strafeIntegral + ki * (error * (currentTime - previousTime)), minI, maxI);
                strafeIntegral = i;
                break;
            case Drive:
                i = Range.clip(driveIntegral + ki * (error * (currentTime - previousTime)), minI, maxI);
                driveIntegral = i;
                break;
            default:
                i = Range.clip(twistIntegral + ki * (error * (currentTime - previousTime)), minI, maxI);
                twistIntegral = i;
        }

        double d;
        switch (which) {
            case Strafe:
                d = kd * ((error - strafeDerivative) / (currentTime - previousTime));
                strafeDerivative = error;
                break;
            case Drive:
                d = kd * ((error - driveDerivative) / (currentTime - previousTime));
                driveDerivative = error;
                break;
            default:
                d = kd * ((error - twistDerivative) / (currentTime - previousTime));
                twistDerivative = error;
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
