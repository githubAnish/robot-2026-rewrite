package org.frogforce503.robot.subsystems.climber;

import org.frogforce503.robot.Robot;

import edu.wpi.first.math.util.Units;

public final class ClimberConstants {
    public static final double kTolerance = Units.inchesToMeters(0.5);

    public static final double minHeight = Robot.bot.getClimberConfig().minHeight();
    public static final double maxHeight = Robot.bot.getClimberConfig().maxHeight();

    public static final double START = minHeight;

    public static final double ALGAE_PLUCK_LOW = START;
    public static final double ALGAE_PLUCK_HIGH = START;
}