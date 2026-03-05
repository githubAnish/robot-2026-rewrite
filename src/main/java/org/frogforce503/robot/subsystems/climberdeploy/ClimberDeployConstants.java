package org.frogforce503.robot.subsystems.climberdeploy;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ClimberDeployConstants {
    // Hardware / Configuration
    public static final int id = 10;
    public static final int limitSwitchId = 0;
    public static final double mechanismRatio = 36.0;

    public static final boolean inverted = false;
    public static final boolean absoluteEncoderInverted = false;
    public static final double absoluteEncoderMechanismRatio = 1.0;
    public static final int statorCurrentLimit = 60;
    public static final double zeroOffset = 0.0;

    public static final PIDConfig kPID = new PIDConfig();
    public static final FFConfig kFF = new FFConfig();
    public static final Constraints kConstraints = new Constraints(0, 0);

    public static final double minAngle = Units.degreesToRadians(0); // TODO Can't say main linkage bar is 0 deg, but can change once block CAD / real CAD comes out
    public static final double maxAngle = Units.degreesToRadians(180); // TODO basically when the 4-bar intake is stowed, the main linkage bar is at 90 deg

    // Setpoints
    public static final double kTolerance = Units.degreesToRadians(3.0); // TODO pivot doesn't need to be as accurate

    public static final double START = maxAngle;

    public static final double STOW = Units.degreesToRadians(89);
    public static final double CLIMB = Units.degreesToRadians(30);
}