package org.frogforce503.robot.subsystems.superstructure.turret;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class TurretConstants {
    // Hardware / Configuration
    public static final int id = 4;
    public static final double mechanismRatio = 100; // currently 6328 turret gear ratio

    public static final boolean inverted = false;
    public static final int statorCurrentLimit = 80;
    public static final double zeroOffset = 0.0;

    public static final PIDConfig kPID = new PIDConfig(2, 0, 0); // some basic pid value
    public static final FFConfig kFF = new FFConfig(0, 0, 2, 0);
    public static final Constraints kConstraints = new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(2700));
    
    public static final double minAngle = Units.degreesToRadians(-210.0); // Assume 6328 params here
    public static final double maxAngle = Units.degreesToRadians(210.0); // Assume 6328 params here
    
    public static final Transform3d robotToTurret = new Transform3d(-0.19685, 0.0, 0.44, Rotation3d.kZero);
    public static final Transform3d turretToCamera =
        new Transform3d(
            -0.1314196, 0.0, 0.2770674, new Rotation3d(0.0, Units.degreesToRadians(-22.5), 0.0));

    // Setpoints
    public static final double kTolerance = Units.degreesToRadians(0.5); // TODO Turret position has to be extremely accurate for consistent shot

    public static final double START = Units.degreesToRadians(0);

    public static final double CLIMB = Units.degreesToRadians(180); // faces backwards
}