package org.frogforce503.robot.subsystems.superstructure.hood;

import org.frogforce503.robot.Robot;

import edu.wpi.first.math.util.Units;

public final class HoodConstants {
    public static final double kTolerance = Units.degreesToRadians(0.5); // Hood position has to be extremely accurate for consistent shot

    public static final double minAngle = Robot.bot.getHoodConfig().minAngle();
    public static final double maxAngle = Robot.bot.getHoodConfig().maxAngle();

    public static final double START = Units.degreesToRadians(45);
}