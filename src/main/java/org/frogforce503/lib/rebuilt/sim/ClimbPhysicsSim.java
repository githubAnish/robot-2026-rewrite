package org.frogforce503.lib.rebuilt.sim;

import org.frogforce503.lib.rebuilt.ClimbUtil;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.climber.ClimberConstants;
import org.frogforce503.robot.subsystems.drive.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * Simulates the robot climbing the tower in the 2026 FRC game "Rebuilt".
 *
 * <p>The climber is on the RIGHT side of the robot (intake = front, climber = right).
 * The robot drives rightward into the tower pole with its bumper slot, then
 * lowers the climber hook to lift itself.
 *
 * <p>This class is purely visual — like {@link BumpPhysicsSim}, it never touches
 * MapleSim internals. It outputs a {@link Pose3d} to be logged in AdvantageScope.
 *
 * <h3>Climb state machine:</h3>
 * <pre>
 *   IDLE
 *     ↓ startClimb()
 *   LATCHING   — robot is near tower, lock X/Y to the climb pose
 *     ↓ (immediately, latch is instantaneous in sim)
 *   CLIMBING   — robot Z rises as climber retracts
 *     ↓ stopClimb()
 *   CLIMBED    — robot frozen at peak height
 * </pre>
 */
public class ClimbPhysicsSim {
    // -------------------------------------------------------------------------
    // Config
    // -------------------------------------------------------------------------

    /**
     * How close the robot must be to the climb pose (meters) before we consider
     * it latched onto the pole. Generous because auto-align is imperfect.
     */
    private static final double LATCH_RADIUS_METERS = Units.inchesToMeters(6.0);

    /**
     * Maximum height the robot CG rises during a full climb (meters).
     * Tune to match the real robot's lift distance.
     */
    private static final double MAX_CLIMB_HEIGHT_METERS = Units.inchesToMeters(10);

    /**
     * How fast the robot rises visually (meters per second of real time).
     * The real climber speed is used to gate this — the robot only rises while
     * the climber is actively retracting.
     */
    private static final double CLIMB_VISUAL_RATE = Units.inchesToMeters(3);

    /**
     * Tilt (roll) applied to the robot while hanging, in radians.
     * Positive = tilt toward the right side (toward the pole).
     * Zero = perfectly level.
     */
    private static final double HANG_ROLL_RAD = Units.degreesToRadians(1.5);

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    private enum ClimbState { IDLE, LATCHING, CLIMBING, CLIMBED }

    private ClimbState state = ClimbState.IDLE;

    /** The field pose the robot snaps to when latched (set at latch time). */
    private Pose2d latchedPose = null;

    /** Current visual Z height of the robot (meters above ground). */
    private double currentHeightMeters = 0.0;

    // -------------------------------------------------------------------------
    // Deps
    // -------------------------------------------------------------------------

    private final Drive drive;
    private final Climber climber;

    public ClimbPhysicsSim(Drive drive, Climber climber) {
        this.drive = drive;
        this.climber = climber;
    }

    // -------------------------------------------------------------------------
    // API (called from GameViz, mirroring old startClimb / climb / stopClimb)
    // -------------------------------------------------------------------------

    /**
     * Called when {@code LowerClimber} begins (i.e. the robot is at the climb pose
     * with the hook extended and now drives into the pole to latch).
     */
    public void startClimb() {
        if (state != ClimbState.IDLE) return;
        state = ClimbState.LATCHING;
    }

    /**
     * Called every loop while {@code LowerClimber} is executing.
     * Advances the state machine and raises the robot Z.
     */
    public void climb() {
        switch (state) {
            case LATCHING -> tryLatch();
            case CLIMBING -> advanceHeight();
            default -> {} // IDLE or CLIMBED — no-op
        }
    }

    /**
     * Called when {@code LowerClimber} ends (climber at goal or interrupted).
     */
    public void stopClimb() {
        if (state == ClimbState.CLIMBING) {
            state = ClimbState.CLIMBED;
        }
        // If interrupted before latch, reset to IDLE so a retry works
        if (state == ClimbState.LATCHING) {
            state = ClimbState.IDLE;
            latchedPose = null;
        }
    }

    /** Resets to initial state (e.g. on match restart). */
    public void reset() {
        state = ClimbState.IDLE;
        latchedPose = null;
        currentHeightMeters = 0.0;
    }

    // -------------------------------------------------------------------------
    // update() — call from GameViz.update() instead of the old climb height code
    // -------------------------------------------------------------------------

    /**
     * Computes the robot's 3D pose for visualization, incorporating climb height
     * and latch snapping.
     *
     * <p>This replaces the raw {@code robotClimbHeightMeters} offset that was in
     * {@code GameViz}. Call this AFTER {@link BumpPhysicsSim#update()} so the
     * bump Pose3d is the base to stack onto.
     *
     * @param bumpPose3d the output of {@link BumpPhysicsSim#update()} for this tick
     * @return the final {@link Pose3d} to log as {@code GameViz/DrivePose3d}
     */
    public Pose3d update(Pose3d bumpPose3d) {
        // Base 2D position: use latched pose when locked, otherwise live odometry
        Pose2d base2d =
            (latchedPose != null)
                ? latchedPose
                : drive.getPose();

        // Build the 3D pose from the base
        Rotation3d baseRotation = bumpPose3d.getRotation(); // preserves bump pitch/roll

        // While hanging, apply a slight roll toward the pole (right side = -Y robot)
        double hangRoll = (state == ClimbState.CLIMBING || state == ClimbState.CLIMBED)
            ? -HANG_ROLL_RAD  // negative = tilt right in robot frame
            : baseRotation.getX();

        Rotation3d finalRotation = new Rotation3d(
            hangRoll,
            baseRotation.getY(),
            base2d.getRotation().getRadians());

        return new Pose3d(
            base2d.getX(),
            base2d.getY(),
            currentHeightMeters,
            finalRotation);
    }

    // -------------------------------------------------------------------------
    // Internal state machine helpers
    // -------------------------------------------------------------------------

    /**
     * Checks if the robot is close enough to a valid climb pose to latch.
     * If so, snaps the visual pose and transitions to CLIMBING.
     */
    private void tryLatch() {
        Pose2d robotPose = drive.getPose();
        Pose2d climbTarget = ClimbUtil.getClimbPose(robotPose);

        double distToClimbPose =
            robotPose.getTranslation().getDistance(climbTarget.getTranslation());

        if (distToClimbPose <= LATCH_RADIUS_METERS) {
            // Snap to the exact climb pose so the robot looks properly aligned
            latchedPose = climbTarget;
            state = ClimbState.CLIMBING;
        }

        // If not close enough yet, stay in LATCHING — the robot is still driving in
    }

    /**
     * Advances the robot's visual height based on real time and climber motion.
     * Only rises while the climber is actively moving (velocity > threshold).
     */
    private void advanceHeight() {
        // Only rise when the climber is actively retracting
        boolean climberMoving = Math.abs(climber.getVelocityMetersPerSec()) > 0.01;

        if (climberMoving) {
            // Scale the visual rise by how far the climber has retracted
            // (normalized 0→1 across its full travel)
            double climberProgress = MathUtil.clamp(
                1.0 - (climber.getHeightMeters() / ClimberConstants.maxHeight),
                0.0,
                1.0);

            // Target height is proportional to climber retraction progress
            double targetHeight = climberProgress * MAX_CLIMB_HEIGHT_METERS;

            // Smoothly approach target height
            currentHeightMeters =
                MathUtil.clamp(
                    currentHeightMeters + CLIMB_VISUAL_RATE * 0.02, // 20ms loop
                    0.0,
                    targetHeight);
        }

        // Clamp to max
        currentHeightMeters = Math.min(currentHeightMeters, MAX_CLIMB_HEIGHT_METERS);
    }

    public boolean hasClimbed() {
        return state == ClimbState.CLIMBED;
    }
}