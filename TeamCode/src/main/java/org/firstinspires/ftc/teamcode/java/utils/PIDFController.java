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
    private static final double maxI = 1;
    private static final double minI = -1;

    //TODO look at using System instead of ElapsedTime
    private final ElapsedTime elapsedTime;
    private final double previousTime;
    private final double ki;
    private final double kp;
    private final double kd;
    private final double f;

    double strafeIntegral = 0;
    double driveIntegral = 0;
    double twistIntegral = 0;

    double strafeDerivative = 0;
    double driveDerivative = 0;
    double twistDerivative = 0;

    public PIDFController(double kp, double ki, double kd, double f) {
        elapsedTime = new ElapsedTime();
        this.previousTime = 0;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.f = f;
    }

    public double[] calculateDrivePowers(double maxV, double xError, double yError, double angleError) {
        double strafe = calculateDrivePID(xError, MovementType.Strafe);
        double drive = calculateDrivePID(yError, MovementType.Drive);
        double twist = calculateDrivePID(angleError, MovementType.Twist);
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

    public double[] calculateDrivePowers(double maxV, MovementData errors) {
        return calculateDrivePowers(maxV, errors.getX(), errors.getY(), errors.getAngle());
    }

    private double calculateDrivePID(double error, MovementType movementType) {
        double currentTime = elapsedTime.time();
        double p = kp * error;
        double i;
        switch (movementType) {
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
        switch (movementType) {
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

    enum MovementType {
        Drive,
        Strafe,
        Twist
    }
}
