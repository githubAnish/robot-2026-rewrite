package org.frogforce503.lib.auto.planned_path.components;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;

import java.util.Optional;

import org.frogforce503.lib.auto.planned_path.PlannedPath.HolonomicState;

/** A trajectory waypoint, including a translation and optional drive/holonomic rotations. */
public class Waypoint {
    @Getter private final Translation2d translation;
    private Rotation2d driveRotation;
    private Rotation2d holonomicRotation;

    /**
     * Constructs a Waypoint with a translation, drive rotation, and holonomic rotation.
     *
     * @param translation Waypoint position (required)
     * @param driveRotation Drive velocity rotation (optional, can be null)
     * @param holonomicRotation Holonomic rotation (optional, can be null)
     */
    public Waypoint(
        Translation2d translation,
        Rotation2d driveRotation,
        Rotation2d holonomicRotation
    ) {
        this.translation = requireNonNullParam(translation, "translation", "Waypoint");
        this.driveRotation = driveRotation;
        this.holonomicRotation = holonomicRotation;
    }

    /**
     * Constructs a Waypoint with a translation (but no drive or holonomic rotation).
     *
     * @param translation Waypoint position (required)
     */
    public Waypoint(Translation2d translation) {
        this.translation = requireNonNullParam(translation, "translation", "Waypoint");
        this.driveRotation = null;
        this.holonomicRotation = null;
    }

    /** Constructs a Waypoint at the origin and without a drive or holonomic rotation. */
    public Waypoint() {
        this(Translation2d.kZero);
    }

    /**
     * Constucts a Waypoint based on a pose.
     *
     * @param pose Source pose (where the rotation describes the drive rotation)
     */
    public static Waypoint fromDifferentialPose(Pose2d pose) {
        requireNonNullParam(pose, "pose", "Waypoint");
        return new Waypoint(pose.getTranslation(), pose.getRotation(), null);
    }

    public Waypoint invertDriveRotation() {
        return
            new Waypoint(
              translation,
              driveRotation
                  .rotateBy(Rotation2d.kPi),
              holonomicRotation);
    }

    /**
     * Constucts a Waypoint based on a pose.
     *
     * @param pose Source pose (where the rotation describes the drive rotation)
     * @param holonomicRotation Holonomic rotation
     */
    public static Waypoint fromDifferentialPose(Pose2d pose, Rotation2d holonomicRotation) {
        requireNonNullParam(pose, "pose", "Waypoint");
        return new Waypoint(pose.getTranslation(), pose.getRotation(), holonomicRotation);
    }

    /**
     * Constucts a Waypoint based on a pose.
     *
     * @param pose Source pose (where the rotation describes the holonomic rotation)
     */
    public static Waypoint fromHolonomicPose(Pose2d pose) {
        requireNonNullParam(pose, "pose", "Waypoint");
        return new Waypoint(pose.getTranslation(), null, pose.getRotation());
    }

    /**
     * Constucts a Waypoint based on a pose.
     *
     * @param pose Source pose (where the rotation describes the holonomic rotation)
     * @param driveRotation Drive rotation
     */
    public static Waypoint fromHolonomicPose(Pose2d pose, Rotation2d driveRotation) {
        requireNonNullParam(pose, "pose", "Waypoint");
        return new Waypoint(pose.getTranslation(), driveRotation, pose.getRotation());
    }
    
    public static Waypoint fromPlannedPathState(HolonomicState state) {
        return new Waypoint(state.poseMeters().getTranslation(), null, state.holonomicAngle());
    }

    public Optional<Rotation2d> getDriveRotation() {
        return Optional.ofNullable(driveRotation);
    }

    public Optional<Rotation2d> getHolonomicRotation() {
        return Optional.ofNullable(holonomicRotation);
    }

    public Waypoint withDriveRotation(Rotation2d newHeading) {
        this.driveRotation = newHeading;
        return this;
    }

    public Waypoint withHolonomicRotation(Rotation2d newHeading) {
        this.holonomicRotation = newHeading;
        return this;
    }

    public Waypoint plus(Translation2d t) {
        return
            new Waypoint(
                translation.plus(t),
                driveRotation,
                holonomicRotation);
    }
}