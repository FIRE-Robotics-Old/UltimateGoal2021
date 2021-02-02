package org.firstinspires.ftc.teamcode.java.util;

public final class GoalPosition {
	public static final double lowGoalHeight = 431.8;
	public static final double middleGoalHeight = 685.8;
	public static final double highGoalHeight = 901.7;
	public static final double powerShotHeight = 596.9;

	public static final double blueGoalY = 866.775;
	public static final double redGoalY = 2701.925;

	public static final double bluePowerShot1Y = 1682.75;
	public static final double bluePowerShot2Y = 1492.25;
	public static final double bluePowerShot3Y = 1301.75;

	public static final double redPowerShot1Y = 1860.55;
	public static final double redPowerShot2Y = 2051.05;
	public static final double redPowerShot3Y = 2241.55;

	public final double height;
	public final double yPosition;
	public final double xPosition = Constants.backFieldX;

	private GoalPosition(double yPosition, double height) {
		this.yPosition = yPosition;
		this.height = height;
	}

	public static GoalPosition generate(Side side, Goal goal) {
		if (side == Side.BLUE) {
			switch (goal) {
				case LOWER_GOAL:
					return new GoalPosition(blueGoalY, lowGoalHeight);
				case MIDDLE_GOAL:
					return new GoalPosition(blueGoalY, middleGoalHeight);
				case HIGH_GOAL:
					return new GoalPosition(blueGoalY, highGoalHeight);
				case POWER_SHOT_1:
					return new GoalPosition(bluePowerShot1Y, powerShotHeight);
				case POWER_SHOT_2:
					return new GoalPosition(bluePowerShot2Y, powerShotHeight);
				default:
					return new GoalPosition(bluePowerShot3Y, powerShotHeight);
			}
		}
		switch (goal) {
			case LOWER_GOAL:
				return new GoalPosition(redGoalY, lowGoalHeight);
			case MIDDLE_GOAL:
				return new GoalPosition(redGoalY, middleGoalHeight);
			case HIGH_GOAL:
				return new GoalPosition(redGoalY, highGoalHeight);
			case POWER_SHOT_1:
				return new GoalPosition(redPowerShot1Y, powerShotHeight);
			case POWER_SHOT_2:
				return new GoalPosition(redPowerShot2Y, powerShotHeight);
			default:
				return new GoalPosition(redPowerShot3Y, powerShotHeight);
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
