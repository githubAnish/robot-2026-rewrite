package org.frogforce503.robot.commands.tuning;

import edu.wpi.first.math.geometry.Rotation2d;

public class WheelRadiusCharacterizationState {
    public double[] positions = new double[4];
    public Rotation2d lastAngle = Rotation2d.kZero;
    public double gyroDelta = 0.0;
}