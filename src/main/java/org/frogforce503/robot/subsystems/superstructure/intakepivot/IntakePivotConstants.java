package org.frogforce503.robot.subsystems.superstructure.intakepivot;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {
    public static final int id = 0;
    public static final double mechanismRatio = 1;

    public static final boolean inverted = false;
    public static final int statorCurrentLimit = 80;
    public static final double zeroOffset = 0.0;

    public static final PIDConfig kPID = new PIDConfig();
    public static final FFConfig kFF = new FFConfig();
    public static final Constraints kConstraints = new Constraints(0, 0);

    public static final double minAngle = Units.degreesToRadians(0); // TODO Can't say main linkage bar is 0 deg, but can change once block CAD / real CAD comes out
    public static final double maxAngle = Units.degreesToRadians(90); // TODO basically when the 4-bar intake is stowed, the main linkage bar is at 90 deg

    public static final double kTolerance = Units.degreesToRadians(3.0); // TODO pivot doesn't need to be as accurate

    public static final double START = maxAngle;

    public static final double STOW = Units.degreesToRadians(89);
    public static final double INTAKE = Units.degreesToRadians(45); 
}