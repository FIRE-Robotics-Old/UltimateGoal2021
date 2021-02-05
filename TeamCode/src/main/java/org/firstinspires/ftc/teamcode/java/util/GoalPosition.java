package org.firstinspires.ftc.teamcode.java.util;

public final class GoalPosition {
	public static final double lowGoalHeight = 431.8;
	public static final double middleGoalHeight = 685.8;
	public static final double highGoalHeight = 901.7;
	public static final double powerShotHeight = 596.9;

	public static final double blueGoalX = 866.775;
	public static final double redGoalX = 2701.925;

	public static final double bluePowerShot1X = 1682.75;
	public static final double bluePowerShot2X = 1492.25;
	public static final double bluePowerShot3X = 1301.75;

	public static final double redPowerShot1X = 1860.55;
	public static final double redPowerShot2X = 2051.05;
	public static final double redPowerShot3X = 2241.55;

	public double height;
	public double xPosition;
	public double yPosition = 1200; //TODO Find right value


	private GoalPosition(double xPosition, double height) {
			this.xPosition = xPosition;
			this.height = height;
	}


	public static GoalPosition generate(Side side, Goal goal) {
		if (side == Side.BLUE) {
			switch (goal) {
				case LOWER_GOAL:
					return new GoalPosition(blueGoalX, lowGoalHeight);
				case MIDDLE_GOAL:
					return new GoalPosition(blueGoalX, middleGoalHeight);
				case HIGH_GOAL:
					return new GoalPosition(blueGoalX, highGoalHeight);
				case POWER_SHOT_1:
					return new GoalPosition(bluePowerShot1X, powerShotHeight);
				case POWER_SHOT_2:
					return new GoalPosition(bluePowerShot2X, powerShotHeight);
				default:
					return new GoalPosition(bluePowerShot3X, powerShotHeight);
			}
		}
		switch (goal) {
			case LOWER_GOAL:
				return new GoalPosition(redGoalX, lowGoalHeight);
			case MIDDLE_GOAL:
				return new GoalPosition(redGoalX, middleGoalHeight);
			case HIGH_GOAL:
				return new GoalPosition(redGoalX, highGoalHeight);
			case POWER_SHOT_1:
				return new GoalPosition(redPowerShot1X, powerShotHeight);
			case POWER_SHOT_2:
				return new GoalPosition(redPowerShot2X, powerShotHeight);
			default:
				return new GoalPosition(redPowerShot3X, powerShotHeight);
		}
	}
}

enum Side {
	BLUE,
	RED,
}

enum Goal {
	LOWER_GOAL,
	MIDDLE_GOAL,
	HIGH_GOAL,
	POWER_SHOT_1,
	POWER_SHOT_2,
	POWER_SHOT_3,
}
