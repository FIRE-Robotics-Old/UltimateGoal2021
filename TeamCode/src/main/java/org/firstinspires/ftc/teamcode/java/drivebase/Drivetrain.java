package org.firstinspires.ftc.teamcode.java.drivebase;

public abstract class Drivetrain {

	protected static final double PI4 = Math.PI / 4;

	public static final double defaultMaxSpeed = 1.0;

	double maxSpeed = defaultMaxSpeed;

	public Drivetrain() {

	}

	public void setMaxSpeed(double maxSpeed) {
		this.maxSpeed = maxSpeed;
	}

	public abstract void stop();

	/**
	 * Scales all wheel speeds to a scaling factor
	 *
	 * @param wheelSpeeds the speeds to scale
	 * @param scaleTo the scaling factor
	 */
	protected void scaleSpeeds(double[] wheelSpeeds, double scaleTo) {
		double maxMagnitude = scalarMax(wheelSpeeds);

		scaleTo = Math.min(maxSpeed, scaleTo);

		// Scales the speeds
		for (int i = 0; i < wheelSpeeds.length; i++) {
			wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude * scaleTo;
		}
	}

	/**
	 * Scales all wheel speeds so that none are greater than the maximum speed.
	 *
	 * @param wheelSpeeds the speeds to scale
	 */
	protected void scaleSpeeds(double[] wheelSpeeds) {
		double maxMagnitude = scalarMax(wheelSpeeds);

		if (maxMagnitude > maxSpeed) {
			for (int i = 0; i < wheelSpeeds.length; i++) {
				wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude * maxSpeed;
			}
		}
	}

	/**
	 * Finds the scalar maximum of an array of doubles
	 *
	 * Scalar Maximum means that only the magnitude, not the sign, will be taken into account
	 * in the calculation
	 * @param values an array of values from which the maximum must be found
	 * @return the maximum
	 * @throws IllegalArgumentException protects against arrays with no elements
	 */
	protected double scalarMax(double[] values) throws IllegalArgumentException {
		if (values.length == 0)
			throw new IllegalArgumentException("There must be at least one value in the array");

		double scalarMax = Math.abs(values[0]);

		for (int i = 1; i < values.length; i++) {
			scalarMax = Math.max(scalarMax, Math.abs(values[i]));
		}

		return scalarMax;
	}

	/**
	 * Finds the vector maximum of an array of doubles
	 *
	 * Vector Maximum means that both the direction (positive/negative) and the magnitude are taken
	 * into account, where positive numbers are all greater than negative numbers.
	 *
	 * @param values an array of values from which the maximum must be found
	 * @return the maximum
	 * @throws IllegalArgumentException protects against arrays with no elements
	 */
	protected double vectorMax(double[] values) throws IllegalArgumentException {
		if (values.length == 0)
			throw new IllegalArgumentException("There must be at least one value in the array");

		double vectorMax = values[0];

		for (int i = 1; i < values.length; i++) {
			vectorMax = Math.max(vectorMax, values[i]);
		}

		return vectorMax;
	}

	/**
	 * Squares Inputs but maintains direction
	 *
	 * This is useful to implement non-linear increase for joystick powers.
	 *
	 * @param input the value to be squared
	 * @return the value squared, maintaining the direction
	 */
	protected double squareInput(double input) {
		return (input < 0 ? -input : input) * input;
	}
}
