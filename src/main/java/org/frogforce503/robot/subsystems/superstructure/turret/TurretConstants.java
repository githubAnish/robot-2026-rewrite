package org.frogforce503.robot.subsystems.superstructure.turret;

import org.frogforce503.robot.Robot;

import edu.wpi.first.math.util.Units;

public final class TurretConstants {
    public static final double kTolerance = Units.degreesToRadians(0.5); // TODO Turret position has to be extremely accurate for consistent shot

    public static final double minAngle = Robot.bot.getTurretConfig().minAngle();
    public static final double maxAngle = Robot.bot.getTurretConfig().maxAngle();

    public static final double START = Units.degreesToRadians(0);

    public static final double CLIMB = Units.degreesToRadians(180); // faces backwards
}