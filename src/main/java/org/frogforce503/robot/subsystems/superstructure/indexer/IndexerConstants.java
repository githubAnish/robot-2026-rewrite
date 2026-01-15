package org.frogforce503.robot.subsystems.superstructure.indexer;

import edu.wpi.first.math.util.Units;

public class IndexerConstants {
    public static final double kTolerance = Units.rotationsPerMinuteToRadiansPerSecond(25.0); // TODO may change based on real robot

    public static final double START = Units.rotationsPerMinuteToRadiansPerSecond(0);

    public static final double SLOW_MIX = Units.rotationsPerMinuteToRadiansPerSecond(500);

    public static final double INTAKE = Units.rotationsPerMinuteToRadiansPerSecond(2000);

    public static final double MIN_FREE_SPEED =
        Units.rotationsPerMinuteToRadiansPerSecond(200); // tune on robot

    public static final double COMPRESSION_CURRENT = 30.0; // amps, tune

}
