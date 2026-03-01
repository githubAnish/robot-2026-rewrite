package org.frogforce503.robot.subsystems.superstructure.hood;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class HoodConstants {
    // Hardware / Configuration
    public static final int id = 6;
    public static final double mechanismRatio = 1;

    public static final boolean inverted = false;
    public static final int statorCurrentLimit = 80;

    public static final PIDConfig kPID = new PIDConfig(0.007, 0, 0.0058);
    public static final FFConfig kFF = new FFConfig();
    public static final Constraints kConstraints = new Constraints(Units.degreesToRadians(480), Units.degreesToRadians(960));
    
    public static final double minAngle = Units.degreesToRadians(0); // TODO 0 deg is when hood horizontal (ball shoots horizontally)
    public static final double maxAngle = Units.degreesToRadians(90);// TODO 90 deg is when hood vertical (ball shoots verticalally)

    public static final Transform3d turretToHood = new Transform3d(0.104, 0, 0.09, Rotation3d.kZero); // Sim turret to hood, can change based on real robot CAD
    
    // Setpoints
    public static final double kFixedTolerance = Units.degreesToRadians(0.5); //TODO Hood position has to be extremely accurate for consistent shot
    public static final double kShootOnMoveTolerance = Units.degreesToRadians(10); //TODO Hood position has to be extremely accurate for consistent shot

    public static final double START = Units.degreesToRadians(0);
    public static final double CLIMB = Units.degreesToRadians(0);

    public static final double BATTER = Units.degreesToRadians(0);
    public static final double LOB_FROM_NZ = Units.degreesToRadians(0);
    public static final double TOWER = Units.degreesToRadians(0);
}