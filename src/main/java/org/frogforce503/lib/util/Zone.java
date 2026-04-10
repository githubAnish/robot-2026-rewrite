package org.frogforce503.lib.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Zone extends Rectangle2d {
    private final Pose2d[] corners =
        new Pose2d[] {
            new Pose2d(getFrontLeftCorner(), getRotation()),
            new Pose2d(getFrontRightCorner(), getRotation()),
            new Pose2d(getBackRightCorner(), getRotation()),
            new Pose2d(getBackLeftCorner(), getRotation()),
            new Pose2d(getFrontLeftCorner(), getRotation()) // circle around to visualize as a full rectangle
        };

    public Zone(Pose2d center, double xWidth, double yWidth) {
        super(center, xWidth, yWidth);
    }

    public Zone(Translation2d cornerA, Translation2d cornerB) {
        super(cornerA, cornerB);
    }

    private Translation2d corner(double xSign, double ySign) {
        Translation2d offset =
            new Translation2d(
                xSign * getXWidth() / 2.0,
                ySign * getYWidth() / 2.0);

        return
            getCenter()
                .getTranslation()
                .plus(offset.rotateBy(getCenter().getRotation()));
    }

    public Translation2d getFrontLeftCorner() {
        return corner(1, 1);
    }

    public Translation2d getFrontRightCorner() {
        return corner(1, -1);
    }

    public Translation2d getBackLeftCorner() {
        return corner(-1, 1);
    }

    public Translation2d getBackRightCorner() {
        return corner(-1, -1);
    }

    public void log(String keyName, Field2d field2d) {
        field2d.getObject(keyName).setPoses(corners);
        Logger.recordOutput(keyName, corners);
    }
}