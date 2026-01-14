package org.frogforce503.robot.subsystems.superstructure.intakepivot;

import org.frogforce503.robot.Robot;

import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {
    public static final double kTolerance = Units.degreesToRadians(3.0); // TODO pivot doesn't need to be as accurate

    public static final double minAngle = Robot.bot.getIntakePivotConfig().minAngle();
    public static final double maxAngle = Robot.bot.getIntakePivotConfig().maxAngle();

    public static final double START = maxAngle;
}