// SuperstructureViz.java
package org.frogforce503.robot.viz;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.launcher.hood.HoodConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;

public class SuperstructureViz {
    // Transforms
    private final Transform3d robotToHood = HoodConstants.robotToHood;
    private final Transform3d robotToIntakePivot = new Transform3d(0.273, 0.0, 0.205, Rotation3d.kZero);
    private final Transform3d robotToLinearExtender = new Transform3d(0.0, 0.0, 0.325, Rotation3d.kZero);
    private final Transform3d robotToVerticalExtender = new Transform3d(0.07, 0.0, 0.0,   Rotation3d.kZero);
    private final Transform3d linearExtenderToDiagonalExtender = new Transform3d(0.35, 0, 0.205, Rotation3d.kZero);
    private final Transform3d robotToClimber = new Transform3d(0.04, -0.34, 0.15, Rotation3d.kZero);
    
    // Linear Extender Constants
    private final double linearExtenderMaxLimit = Units.inchesToMeters(10);

    // Vertical Extender Constants
    private final double verticalExtenderMaxLift = Units.inchesToMeters(5);

    private final double verticalExtenderRiseRate = Units.inchesToMeters(2);
    private final double verticalExtenderFallRate = Units.inchesToMeters(5);

    private final int minFuelForVerticalLifeActivation = 35;
    private final int maxFuelForVerticalLifeActivation = 70;

    // Diagonal Extender Constants
    private final double diagonalExtenderAngleOffsetRad = Units.degreesToRadians(30);

    private final double diagonalExtenderMaxAngle = Units.degreesToRadians(20);

    private final double diagonalRiseRate = Units.degreesToRadians(10.0);
    private final double diagonalFallRate = Units.degreesToRadians(5.0);

    private final int minFuelForDiagonalPressureActivation = 35;
    private final int maxFuelForDiagonalPressureActivation = 70;

    // State
    @Getter private double currentLinearExtensionX = 0.0;
    @Getter private double currentVerticalLift = 0.0;
    @Getter private double currentDiagonalAngleRad = 0.0;
    
    private final Timer timer = new Timer();
    private boolean timerStarted = false;

    public void update(
        Pose3d drivePose3d,
        double hoodAngleRad,
        double intakePivotAngleRad,
        double climberHeightMeters,
        int numFuelInRobot
    ) {
        // ── Delta time ────────────────────────────────────────────────────────────
        double dt;
        
        if (!timerStarted) {
            timer.start();
            timerStarted = true;
            dt = Constants.loopPeriodSecs;
            
        } else {
            dt = timer.get();
            timer.reset();
            timer.start();
            dt = MathUtil.clamp(dt, 0.005, 0.1); // guard against spikes
        }

        double normalizedPivot = getNormalizedIntakePivotAngleRad(intakePivotAngleRad);

        // ── 1. LINEAR HOPPER EXTENSION ────────────────────────────────────────────
        // // Follows intake pivot outward — ratcheted, never retracts
        // double desiredLinearX = ;
        currentLinearExtensionX = Math.max(currentLinearExtensionX, linearExtenderMaxLimit * normalizedPivot);

        // ── 2. VERTICAL HOPPER EXTENSION ─────────────────────────────────────────
        // Target lift is determined purely by ball count
        double targetVerticalLift =
            verticalExtenderMaxLift *
            pressureRamp(numFuelInRobot, minFuelForVerticalLifeActivation, maxFuelForVerticalLifeActivation);

        if (currentVerticalLift < targetVerticalLift) {
            // Balls pushing it up
            currentVerticalLift =
                Math.min(
                    currentVerticalLift + verticalExtenderRiseRate * dt,
                    targetVerticalLift);
        } else {
            // Gravity pulling it down as balls leave
            currentVerticalLift =
                Math.max(
                    currentVerticalLift - verticalExtenderFallRate * dt,
                    targetVerticalLift);
        }

        currentVerticalLift = MathUtil.clamp(currentVerticalLift, 0, verticalExtenderMaxLift);

        // ── 3. DIAGONAL HOPPER EXTENSION ─────────────────────────────────────────
        double targetDiagonalAngle =
            diagonalExtenderMaxAngle *
            pressureRamp(numFuelInRobot, minFuelForDiagonalPressureActivation, maxFuelForDiagonalPressureActivation);

        // Below threshold — no balls touching it, snaps flat immediately
        if (numFuelInRobot <= minFuelForDiagonalPressureActivation) {
            currentDiagonalAngleRad = 0.0;

        } else if (currentDiagonalAngleRad < targetDiagonalAngle) {
            // Ball pressure pushing diagonal up
            currentDiagonalAngleRad =
                Math.min(
                    currentDiagonalAngleRad + diagonalRiseRate * dt,
                    targetDiagonalAngle);

        } else {
            // Balls leaving — slowly returns down
            currentDiagonalAngleRad =
                Math.max(
                    currentDiagonalAngleRad - diagonalFallRate * dt,
                    targetDiagonalAngle);
        }

        currentDiagonalAngleRad = MathUtil.clamp(currentDiagonalAngleRad, 0.0, diagonalExtenderMaxAngle);

        // Calculate poses
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
                .plus(robotToLinearExtender)
                .plus(new Transform3d(new Translation3d(currentLinearExtensionX, 0, 0), Rotation3d.kZero));

        var verticalExtenderExtenderPose =
            Pose3d.kZero
                .plus(robotToVerticalExtender)
                .plus(new Transform3d(new Translation3d(0, 0, currentVerticalLift), Rotation3d.kZero));

        var diagonalExtenderPose =
            linearHopperExtenderPose
                .plus(linearExtenderToDiagonalExtender)
                .plus(new Transform3d(Translation3d.kZero,
                    new Rotation3d(0.0, -currentDiagonalAngleRad + diagonalExtenderAngleOffsetRad, Math.PI)));

        var climberPose =
            Pose3d.kZero
                .plus(robotToClimber)
                .plus(new Transform3d(new Translation3d(0, 0, climberHeightMeters * 0.53), Rotation3d.kZero));

        Logger.recordOutput(
            "SuperstructureViz/Components",
            hoodPose,
            intakePivotPose,
            linearHopperExtenderPose, verticalExtenderExtenderPose, diagonalExtenderPose,
            climberPose);
    }

    private double pressureRamp(int numBalls, int startBalls, int fullBalls) {
        if (numBalls <= startBalls) {
            return 0.0;
        }

        if (numBalls >= fullBalls) {
            return 1.0;
        }

        return (double) (numBalls - startBalls) / (fullBalls - startBalls);
    }

    private double getNormalizedIntakePivotAngleRad(double intakePivotAngleRad) {
        double clamped = MathUtil.clamp(intakePivotAngleRad, 0, Math.PI / 2);
        return (Math.PI / 2 - clamped) / (Math.PI / 2);
    }

    
}