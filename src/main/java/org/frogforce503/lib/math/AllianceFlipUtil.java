package org.frogforce503.lib.math;

import java.util.function.UnaryOperator;

import org.frogforce503.lib.util.Zone;
import org.frogforce503.robot.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class AllianceFlipUtil {
    private static boolean override;

    private AllianceFlipUtil() {}

    private static boolean shouldFlip() {
        return FieldConstants.isRed() || override;
    }

    /** Applies a flip without depending on the current alliance. */
    public static <T> T flip(UnaryOperator<T> flipFunction, T blueValue) {
        try {
            override = true;
            return flipFunction.apply(blueValue);
        } finally {
            override = false;
        }
    }

    public static double applyX(double x) {
        return shouldFlip() ? FieldConstants.fieldLength - x : x;
    }

    public static double applyY(double y) {
        return shouldFlip() ? FieldConstants.fieldWidth - y : y;
    }

    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    public static Pose2d apply(Pose2d pose) {
        return
            shouldFlip()
                ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
                : pose;
    }

    public static Translation3d apply(Translation3d translation) {
        return
            new Translation3d(
                applyX(translation.getX()),
                applyY(translation.getY()),
                translation.getZ());
    }

    public static Rotation3d apply(Rotation3d rotation) {
        return
            shouldFlip()
                ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI))
                : rotation;
    }

    public static Pose3d apply(Pose3d pose) {
        return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
    }

    public static Zone apply(Zone zone) {
        return
            new Zone(
                apply(zone.getCenter()),
                zone.getXWidth(),
                zone.getYWidth());
    }
}