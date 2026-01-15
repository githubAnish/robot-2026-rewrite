package org.frogforce503.robot.subsystems.superstructure.intakeroller;

import edu.wpi.first.math.util.Units;

public class IntakeRollerConstants {
    public static final double kTolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0); // TODO may change based on real robot

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);

    public static final double INTAKE = Units.rotationsPerMinuteToRadiansPerSecond(2000);
}
