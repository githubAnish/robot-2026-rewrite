// SuperstructureViz.java
package org.frogforce503.robot.viz;

import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class SuperstructureViz {
    private final Transform3d robotToHood = HoodConstants.robotToHood;
    private final Transform3d robotToIntakePivot      = new Transform3d(0.273, 0.0, 0.205, Rotation3d.kZero);
    private final Transform3d robotToLinearHopperExtender   = new Transform3d(0.0, 0.0, 0.325, Rotation3d.kZero);
    private final Transform3d robotToVerticalHopperExtender = new Transform3d(0.07, 0.0, 0.0,   Rotation3d.kZero);
    private final Transform3d linearHopperExtenderToDiagonalHopperExtender = new Transform3d(0.35, 0, 0.205, Rotation3d.kZero);
    private final Transform3d robotToClimber = new Transform3d(0.04, -0.34, 0.15, Rotation3d.kZero);

    // ── Tunable limits ──────────────────────────────────────────────────────────
    public static final double linearHopperExtenderMaxLimit    = Units.inchesToMeters(10);
    public static final double diagonalHopperExtenderMaxAngle  = Units.degreesToRadians(20);
    public static final double verticalHopperExtenderMaxLift   = Units.inchesToMeters(5);

    // ── Ball-count thresholds (tune here) ───────────────────────────────────────
    public static final int DIAGONAL_PRESSURE_START_BALLS = 45; // was 6
    public static final int DIAGONAL_PRESSURE_FULL_BALLS  = 70; // was 16
    public static final int VERTICAL_LIFT_START_BALLS     = 45;
    public static final int VERTICAL_LIFT_FULL_BALLS      = 70;

    // ── Physics rates (tune here) ────────────────────────────────────────────────
    // How fast the vertical extender rises/falls (meters per second)
    private static final double VERTICAL_RISE_RATE   = Units.inchesToMeters(1.5); // per second, driven by balls
    private static final double VERTICAL_GRAVITY_RATE = Units.inchesToMeters(4); // was 1.5 — gravity is fast

    // How fast diagonal angle changes (radians per second)
    private static final double DIAGONAL_RISE_RATE   = Units.degreesToRadians(10.0);
    private static final double DIAGONAL_RETURN_RATE = Units.degreesToRadians(5.0);

    // ── Stateful values ──────────────────────────────────────────────────────────
    private double currentLinearExtensionX  = 0.0;   // ratcheted — never decreases
    private double currentVerticalLift      = 0.0;   // smooth gravity return
    private double currentDiagonalAngleRad  = 0.0;   // smooth pressure/return

    private final Timer updateTimer = new Timer();
    private boolean timerStarted = false;

    public void update(Pose3d drivePose3d, double hoodAngleRad, double intakePivotAngleRad, int numFuelInRobot) {
        // ── Delta time ────────────────────────────────────────────────────────────
        double dt;
        if (!timerStarted) {
            updateTimer.start();
            timerStarted = true;
            dt = 0.02;
        } else {
            dt = updateTimer.get();
            updateTimer.reset();
            updateTimer.start();
            dt = MathUtil.clamp(dt, 0.005, 0.1); // guard against spikes
        }

        double normalizedPivot = getNormalizedIntakePivotAngleRad(intakePivotAngleRad);

        // ── 1. LINEAR HOPPER EXTENSION ────────────────────────────────────────────
        // Follows intake pivot outward — ratcheted, never retracts
        double desiredLinearX = linearHopperExtenderMaxLimit * normalizedPivot;
        currentLinearExtensionX = Math.max(currentLinearExtensionX, desiredLinearX);

        // ── 2. VERTICAL HOPPER EXTENSION ─────────────────────────────────────────
        // Target lift is determined purely by ball count
        double targetVerticalLift = verticalHopperExtenderMaxLift
            * pressureRamp(numFuelInRobot, VERTICAL_LIFT_START_BALLS, VERTICAL_LIFT_FULL_BALLS);

        if (currentVerticalLift < targetVerticalLift) {
            // Balls pushing it up
            currentVerticalLift = Math.min(
                currentVerticalLift + VERTICAL_RISE_RATE * dt,
                targetVerticalLift);
        } else {
            // Gravity pulling it down as balls leave
            currentVerticalLift = Math.max(
                currentVerticalLift - VERTICAL_GRAVITY_RATE * dt,
                targetVerticalLift);
        }
        currentVerticalLift = MathUtil.clamp(currentVerticalLift, 0, verticalHopperExtenderMaxLift);

        // ── 3. DIAGONAL HOPPER EXTENSION ─────────────────────────────────────────
        double targetDiagonalAngle = diagonalHopperExtenderMaxAngle
            * pressureRamp(numFuelInRobot, DIAGONAL_PRESSURE_START_BALLS, DIAGONAL_PRESSURE_FULL_BALLS);

        // Below threshold — no balls touching it, snaps flat immediately
        if (numFuelInRobot <= DIAGONAL_PRESSURE_START_BALLS) {
            currentDiagonalAngleRad = 0.0;
        } else if (currentDiagonalAngleRad < targetDiagonalAngle) {
            // Ball pressure pushing diagonal up
            currentDiagonalAngleRad = Math.min(
                currentDiagonalAngleRad + DIAGONAL_RISE_RATE * dt,
                targetDiagonalAngle);
        } else {
            // Balls leaving — slowly returns down
            currentDiagonalAngleRad = Math.max(
                currentDiagonalAngleRad - DIAGONAL_RETURN_RATE * dt,
                targetDiagonalAngle);
        }
        currentDiagonalAngleRad = MathUtil.clamp(currentDiagonalAngleRad, 0.0, diagonalHopperExtenderMaxAngle);

        // ── Compute poses ─────────────────────────────────────────────────────────
        var hoodPose =
            Pose3d.kZero
                .plus(robotToHood)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, hoodAngleRad, Math.PI)));

        var intakePivotPose =
            Pose3d.kZero
                .plus(robotToIntakePivot)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -intakePivotAngleRad, 0)));

        var linearHopperExtenderPose =
            Pose3d.kZero
                .plus(robotToLinearHopperExtender)
                .plus(new Transform3d(new Translation3d(currentLinearExtensionX, 0, 0), Rotation3d.kZero));

        var verticalHopperExtenderPose =
            Pose3d.kZero
                .plus(robotToVerticalHopperExtender)
                .plus(new Transform3d(new Translation3d(0, 0, currentVerticalLift), Rotation3d.kZero));

        var diagonalHopperExtenderPose =
            linearHopperExtenderPose
                .plus(linearHopperExtenderToDiagonalHopperExtender)
                .plus(new Transform3d(Translation3d.kZero,
                    new Rotation3d(0.0, -currentDiagonalAngleRad + Units.degreesToRadians(30), Math.PI)));

        var climberPose =
            Pose3d.kZero
                .plus(robotToClimber)
                .plus(new Transform3d(new Translation3d(0, 0, 0.0), Rotation3d.kZero));

        Logger.recordOutput(
            "SuperstructureViz/Components",
            hoodPose,
            intakePivotPose,
            linearHopperExtenderPose, verticalHopperExtenderPose, diagonalHopperExtenderPose,
            climberPose);

        // Debug logging so you can tune thresholds in AdvantageScope
        Logger.recordOutput("SuperstructureViz/LinearExtensionM",   currentLinearExtensionX);
        Logger.recordOutput("SuperstructureViz/VerticalLiftM",       currentVerticalLift);
        Logger.recordOutput("SuperstructureViz/DiagonalAngleDeg",    Units.radiansToDegrees(currentDiagonalAngleRad));
    }

    /** Maps ball count linearly 0→1 between [startBalls, fullBalls]. */
    private double pressureRamp(int numBalls, int startBalls, int fullBalls) {
        if (numBalls <= startBalls) return 0.0;
        if (numBalls >= fullBalls)  return 1.0;
        return (double)(numBalls - startBalls) / (fullBalls - startBalls);
    }

    private double getNormalizedIntakePivotAngleRad(double intakePivotAngleRad) {
        double clamped = MathUtil.clamp(intakePivotAngleRad, 0, Math.PI / 2);
        return (Math.PI / 2 - clamped) / (Math.PI / 2);
    }

    // ── Getters so FuelViz can mirror the exact same state ────────────────────
    public double getCurrentLinearExtensionX()  { return currentLinearExtensionX; }
    public double getCurrentVerticalLift()       { return currentVerticalLift; }
    public double getCurrentDiagonalAngleRad()   { return currentDiagonalAngleRad; }
}