package org.frogforce503.robot.subsystems.superstructure.flywheels;

import edu.wpi.first.math.util.Units;

public class FlywheelsConstants {
    public static final double kRollerTolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0); // Flywheel speed has to be extremely accurate for consistent shot

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);
}
