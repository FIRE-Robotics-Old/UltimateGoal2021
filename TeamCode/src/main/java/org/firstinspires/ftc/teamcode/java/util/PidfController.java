package org.firstinspires.ftc.teamcode.java.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * the PIDController class will be used for  all the different pid calculations
 * for example AutoDriving ,autoAdjusting , controlling the shooter speed
 */
//P is main power, I looks at the sum of error and gives final push, D is how much the error is changing
public class PidfController {
	private static final double maxI = .1;
	private static final double minI = -.1;

	//TODO look at using System instead of ElapsedTime
	private final ElapsedTime elapsedTime;
	private final double previousTime;
	private final double ki;
	private final double kp;
	private final double kd;
	private final double f;

	double integral = 0;

	double derivative = 0;

	public PidfController(double kp, double ki, double kd, double f) {
		elapsedTime = new ElapsedTime();
		this.previousTime = 0;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.f = f;
	}

	public double calculatePID(double error) {
		double currentTime = elapsedTime.nanoseconds();
		double p = kp * error;
		double i = Range.clip(integral + ki * (error * (currentTime - previousTime)), minI, maxI);
		integral = i;
		double d = kd * ((error - derivative) / (currentTime - previousTime));
		derivative = error;

		return f + (1 - f) * (p + i + d); //Scales
	}
}
