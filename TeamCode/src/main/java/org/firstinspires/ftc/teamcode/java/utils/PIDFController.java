package org.firstinspires.ftc.teamcode.java.utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    private double aError;

    public PIDFController(double kp, double ki, double kd, double f) {
        elapsedTime = new ElapsedTime();
        this.previousTime = 0;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.f = f;
    }

    public double[] calculateDrivePowers(double maxV, double xError, double yError, double angleError) {
        double currentTime = elapsedTime.time();
        aError = angleError;
        angleError = ((Math.toDegrees(angleError) + 360) % 360);
        double strafe = calculateDrivePID(xError, MovementType.Strafe, currentTime);
        double drive = calculateDrivePID(yError, MovementType.Drive, currentTime);
        double twist = calculateDrivePID(angleError, MovementType.Twist, currentTime);

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };
        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (Math.abs(speed) > max) {
                max = Math.abs(speed);
            }
        }

        // Finds the max after converting doubles to Doubles
        //double max = Collections.max(Arrays.stream(speeds).boxed().collect(Collectors.toList()));


        if (max > maxV)
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] *= maxV / max;
            }

        return speeds;
    }

    public double[] calculateDrivePowers(double maxV, MovementData errors) {
        return calculateDrivePowers(maxV, errors.getX(), errors.getY(), errors.getAngle());
    }

    private double calculateDrivePID(double error, MovementType movementType, double currentTime) {
        double p = kp * error;
        double i = updateIntegral(error, movementType, currentTime);
        double d = updateDerivative(error, movementType, currentTime);

        return f + (1 - f) * (p + i + d); //Scales
    }

    private double updateDerivative(double error, MovementType movementType, double currentTime) {
        double d;

        // TODO decide to use single line and faster but harder to read or easier to read but slower
        // double errorDiff = error - (movementType == MovementType.Strafe ? (strafeDerivative + (0 * (strafeDerivative = error))) : (movementType == MovementType.Drive ? (driveDerivative + (0 * (driveDerivative = error))) : twistDerivative + (0 * (twistDerivative = error))));
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
        return d;
    }

    private double updateIntegral(double error, MovementType movementType, double currentTime) {
        double integral;
        switch (movementType) {
            case Strafe:
                integral = Range.clip(strafeIntegral + ki * (error * (currentTime - previousTime)), minI, maxI);
                strafeIntegral = integral;
                break;
            case Drive:
                integral = Range.clip(driveIntegral + ki * (error * (currentTime - previousTime)), minI, maxI);
                driveIntegral = integral;
                break;
            default:
                integral = Range.clip(twistIntegral + ki * (error * (currentTime - previousTime)), minI, maxI);
                twistIntegral = integral;
        }
        return integral;
    }


    enum MovementType {
        Drive,
        Strafe,
        Twist
    }


    public double getAngleErrorDegrees() {
        return ((Math.toDegrees(aError) + 360) % 360);
    }
}
