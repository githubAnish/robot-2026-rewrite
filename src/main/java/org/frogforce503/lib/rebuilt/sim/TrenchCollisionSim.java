package org.frogforce503.lib.rebuilt.sim;

import org.frogforce503.lib.math.AllianceFlipUtil;
import org.frogforce503.lib.util.Zone;
import org.frogforce503.robot.FieldConstants;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.viz.SuperstructureViz;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * TrenchCollisionSim
 *
 * <p>Simulates the physical constraint that a robot with its vertical hopper raised
 * cannot pass underneath a trench. When the vertical hopper lift is at or above
 * {@link #HOPPER_HEIGHT_BLOCKED_THRESHOLD_M}, any chassis velocity component that
 * would move the robot deeper into a trench opening is zeroed out, while the
 * perpendicular component (and any component moving away from the trench) is
 * left untouched.
 *
 * <p>This class does NOT call drive.runVelocity() itself. Instead it exposes
 * {@link #filterSpeeds(ChassisSpeeds, Pose2d, Rotation2d)} which Drive calls
 * internally inside its own runVelocity, so every caller is covered with no fighting.
 *
 * <p>All four trenches are supported (blue-left, blue-right, red-left, red-right)
 * and both the alliance-zone side and the neutral-zone side of each trench are
 * handled independently.
 */
public class TrenchCollisionSim {
    private final Zone redLeftTrench = AllianceFlipUtil.flip(AllianceFlipUtil::apply, FieldConstants.Trench.blueLeft);
    private final Zone redRightTrench = AllianceFlipUtil.flip(AllianceFlipUtil::apply, FieldConstants.Trench.blueRight);

    // ── Threshold ────────────────────────────────────────────────────────────────

    /** Vertical hopper height at which the robot can no longer pass under a trench. */
    public static final double HOPPER_HEIGHT_BLOCKED_THRESHOLD_M = Units.inchesToMeters(2.0);

    // ── Trench geometry (from FFArena2026Rebuilt) ─────────────────────────────────
    //
    // Each trench wall is 53 in tall x 12 in wide, centred at:
    //   X: 8.27 +/- trenchWallDistX   (trenchWallDistX = 120 in + 47/2 in = 143.5 in -> 3.6449 m)
    //   Y: 4.035 +/- trenchWallDistY  (trenchWallDistY = 73 in + 47/2 in + 6 in = 102.5 in -> 2.6035 m)

    private static final double FIELD_CENTER_X = 8.27;

    private static final double TRENCH_WALL_DIST_X =
            Units.inchesToMeters(120.0) + Units.inchesToMeters(47.0 / 2.0); // ~3.6449 m

    private static final double TRENCH_HALF_HEIGHT_Y = Units.inchesToMeters(53.0) / 2.0; // ~0.6731 m

    // Extra margin so we stop a little before the actual wall face
    private static final double APPROACH_MARGIN = Units.inchesToMeters(0.0);

    private final double robotHalfLength = DriveConstants.bumperLength / 2.0 + APPROACH_MARGIN;
    private final double robotHalfWidth  = DriveConstants.bumperWidth  / 2.0;

    // ── Cached trench list (built once) ──────────────────────────────────────────

    private final TrenchInfo[] trenches = buildTrenches();

    // ── Dependency ───────────────────────────────────────────────────────────────

    private final SuperstructureViz superstructureViz;

    public TrenchCollisionSim(SuperstructureViz superstructureViz) {
        this.superstructureViz = superstructureViz;
    }

    // ── Public API ────────────────────────────────────────────────────────────────

    /**
     * Filters robot-relative {@link ChassisSpeeds} so the robot cannot drive into
     * a trench when the vertical hopper is raised.
     *
     * <p>Called by {@code Drive.runVelocity()} before forwarding to the IO layer,
     * so every caller (teleop, auto, commands) is covered automatically.
     *
     * @param robotRelativeSpeeds the speeds that were requested (robot-relative)
     * @param pose                the robot's current field pose
     * @param rotation            the robot's current heading
     * @return filtered robot-relative speeds, with any trench-penetrating component zeroed
     */
    public ChassisSpeeds filterSpeeds(ChassisSpeeds robotRelativeSpeeds, Pose2d pose, Rotation2d rotation) {
        double verticalLift = superstructureViz.getCurrentVerticalLift();
        boolean hopperRaised = verticalLift >= HOPPER_HEIGHT_BLOCKED_THRESHOLD_M;

        Logger.recordOutput("TrenchCollisionSim/HopperRaised", hopperRaised);

        if (!hopperRaised) {
            return robotRelativeSpeeds; // Robot fits under the trench — no restriction
        }

        // Convert to field-relative so we can reason about X/Y geometry
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, rotation);

        ChassisSpeeds filteredField = applyTrenchFilter(pose.getTranslation(), fieldSpeeds);

        // Convert back to robot-relative for the caller
        return ChassisSpeeds.fromFieldRelativeSpeeds(filteredField, rotation);
    }

    // ── Private helpers ───────────────────────────────────────────────────────────

    /**
     * Core filter: iterates all four trenches and zeroes any field-relative vx
     * component that would push the robot through a blocked trench wall.
     */
    private ChassisSpeeds applyTrenchFilter(Translation2d pos, ChassisSpeeds fieldSpeeds) {
        double vx    = fieldSpeeds.vxMetersPerSecond;
        double vy    = fieldSpeeds.vyMetersPerSecond;
        double omega = fieldSpeeds.omegaRadiansPerSecond;

        for (TrenchInfo trench : trenches) {
            if (!robotIsInTrenchYRange(pos, trench)) {
                continue; // Not laterally aligned with this trench — skip
            }

            // ── Alliance-zone side wall ───────────────────────────────────────────
            // distToWall > 0 means the robot is on the outside (hasn't entered yet),
            // measured along the inward direction.
            double distToAllianceWall = trench.inwardSignX * (trench.allianceSideX - pos.getX());
            if (distToAllianceWall > -robotHalfLength && distToAllianceWall < robotHalfLength) {
                double inwardVx = vx * trench.inwardSignX;
                if (inwardVx > 0) {
                    vx = 0;
                    Logger.recordOutput("TrenchCollisionSim/BlockedAt", "AllianceSide_" + trench.name);
                }
            }

            // ── Neutral-zone side wall ────────────────────────────────────────────
            double distToNeutralWall = -trench.inwardSignX * (trench.neutralSideX - pos.getX());
            if (distToNeutralWall > -robotHalfLength && distToNeutralWall < robotHalfLength) {
                double inwardVx = vx * (-trench.inwardSignX);
                if (inwardVx > 0) {
                    vx = 0;
                    Logger.recordOutput("TrenchCollisionSim/BlockedAt", "NeutralSide_" + trench.name);
                }
            }
        }

        return new ChassisSpeeds(vx, vy, omega);
    }

    /** Returns true if the robot is laterally inside the Y span of this trench opening. */
    private boolean robotIsInTrenchYRange(Translation2d pos, TrenchInfo trench) {
        double yMin = trench.centerY - TRENCH_HALF_HEIGHT_Y - robotHalfWidth;
        double yMax = trench.centerY + TRENCH_HALF_HEIGHT_Y + robotHalfWidth;
        return pos.getY() >= yMin && pos.getY() <= yMax;
    }

    /**
     * Builds the four trench wall descriptors from field constants.
     *
     * <p>Wall width along X is 12 in. Wall centres are at
     * FIELD_CENTER_X +/- TRENCH_WALL_DIST_X. For blue trenches the
     * "inward" direction is +X (robot enters from the low-X alliance side);
     * for red trenches it is -X.
     */
    private TrenchInfo[] buildTrenches() {
        final double wallHalfWidthX = Units.inchesToMeters(12.0) / 2.0; // 0.1524 m

        double blueWallCenterX   = FIELD_CENTER_X - TRENCH_WALL_DIST_X;
        double blueAllianceFaceX = blueWallCenterX - wallHalfWidthX; // face closer to blue wall
        double blueNeutralFaceX  = blueWallCenterX + wallHalfWidthX; // face closer to field centre

        double redWallCenterX   = FIELD_CENTER_X + TRENCH_WALL_DIST_X;
        double redAllianceFaceX = redWallCenterX + wallHalfWidthX;   // face closer to red wall
        double redNeutralFaceX  = redWallCenterX - wallHalfWidthX;   // face closer to field centre

        double blueLowerTrenchY = FieldConstants.Trench.blueRight.getCenter().getY();
        double blueUpperTrenchY = FieldConstants.Trench.blueLeft.getCenter().getY();
        double redLowerTrenchY  = redRightTrench.getCenter().getY();
        double redUpperTrenchY  = redLeftTrench.getCenter().getY();

        return new TrenchInfo[] {
            new TrenchInfo("BlueRight", blueAllianceFaceX, blueNeutralFaceX, +1, blueLowerTrenchY),
            new TrenchInfo("BlueLeft",  blueAllianceFaceX, blueNeutralFaceX, +1, blueUpperTrenchY),
            new TrenchInfo("RedRight",  redAllianceFaceX,  redNeutralFaceX,  -1, redLowerTrenchY),
            new TrenchInfo("RedLeft",   redAllianceFaceX,  redNeutralFaceX,  -1, redUpperTrenchY),
};
    }

    // ── Data record ──────────────────────────────────────────────────────────────

    private record TrenchInfo(
            String name,
            double allianceSideX,
            double neutralSideX,
            double inwardSignX,
            double centerY) {}
}