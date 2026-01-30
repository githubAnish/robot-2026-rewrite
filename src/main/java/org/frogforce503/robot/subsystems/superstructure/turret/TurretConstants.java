package org.frogforce503.robot.subsystems.superstructure.turret;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class TurretConstants {
    public static final int id = 4;
    public static final double mechanismRatio = 100; // currently 6328 turret gear ratio

    public static final boolean inverted = false;
    public static final int statorCurrentLimit = 80;
    public static final double zeroOffset = 0.0;

    public static final PIDConfig kPID = new PIDConfig(2, 0, 0); // some basic pid value
    public static final FFConfig kFF = new FFConfig(0, 0, 2, 0);
    public static final Constraints kConstraints = new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(2700));
    
    public static final double minAngle = Units.degreesToRadians(-210); // TODO assume 6328 min angle for now;
    public static final double maxAngle = Units.degreesToRadians(210); // TODO assume 6328 min angle for now;

    public static final double kTolerance = Units.degreesToRadians(0.5); // TODO Turret position has to be extremely accurate for consistent shot

    public static final double START = Units.degreesToRadians(0);

    public static final double CLIMB = Units.degreesToRadians(180); // faces backwards
}