package org.firstinspires.ftc.teamcode.java.util;

public final class GoalPosition {
    private double height;
    private double yPosition;

    private GoalPosition(double yPosition, double height) {
        this.yPosition = yPosition;
        this.height = height;
    }

    public static GoalPosition generate(Side s, Goal g) {
        if (s == Side.BLUE) {
            switch (g) {
                case LOWER_GOAL:
                    return new GoalPosition(Constants.blueGoalY, Constants.lowGoalHeight);
                case MIDDLE_GOAL:
                    return new GoalPosition(Constants.blueGoalY, Constants.middleGoalHight);
                case HIGH_GOAL:
                    return new GoalPosition(Constants.blueGoalY, Constants.highGoalHeight);
                case POWER_SHOT_1:
                    return new GoalPosition(Constants.bluePowerShot1Y, Constants.powerShotHeight);
                case POWER_SHOT_2:
                    return new GoalPosition(Constants.bluePowerShot2Y, Constants.powerShotHeight);
                default:
                    return new GoalPosition(Constants.bluePowerShot3Y, Constants.powerShotHeight);
            }
        }
        switch (g) {
            case LOWER_GOAL:
                return new GoalPosition(Constants.redGoalY, Constants.lowGoalHeight);
            case MIDDLE_GOAL:
                return new GoalPosition(Constants.redGoalY, Constants.middleGoalHight);
            case HIGH_GOAL:
                return new GoalPosition(Constants.redGoalY, Constants.highGoalHeight);
            case POWER_SHOT_1:
                return new GoalPosition(Constants.redPowerShot1Y, Constants.powerShotHeight);
            case POWER_SHOT_2:
                return new GoalPosition(Constants.redPowerShot2Y, Constants.powerShotHeight);
            default:
                return new GoalPosition(Constants.redPowerShot3Y, Constants.powerShotHeight);
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
