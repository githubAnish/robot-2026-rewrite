package org.frogforce503.lib.auto.planned_path;

import java.util.ArrayList;
import java.util.List;

import org.frogforce503.lib.auto.planned_path.components.RotationSequence;
import org.frogforce503.lib.auto.planned_path.components.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import lombok.Getter;

public class PlannedPath {
    @Getter private Trajectory driveTrajectory;
    @Getter private RotationSequence rotationSequence;
    @Getter private List<Waypoint> waypoints; 

    public PlannedPath(Trajectory driveTrajectory, RotationSequence rotationSequence, List<Waypoint> waypoints) {
        this.driveTrajectory = driveTrajectory;
        this.rotationSequence = rotationSequence;
        this.waypoints = waypoints;
    }

    public PlannedPath() {
        this(
            new Trajectory(),
            new RotationSequence(),
            List.of());
    }

    public Pose2d getInitialHolonomicPose() {
        Pair<Trajectory.State, RotationSequence.State> sample = _sample(0);

        return
            new Pose2d(
                sample.getFirst().poseMeters.getTranslation(),
                sample.getSecond().position);
    }

    public Pose2d getFinalHolonomicPose() {
        Pair<Trajectory.State, RotationSequence.State> sample = _sample(getTotalTimeSeconds());
        
        return
            new Pose2d(
                sample.getFirst().poseMeters.getTranslation(),
                sample.getSecond().position);    
    }

    public Waypoint getFinalWaypoint() {
        return this.waypoints.get(this.waypoints.size() - 1);
    }

    public List<Waypoint> sampleTimeRange(double ti, double tf, double intervals) {
        List<Waypoint> result = new ArrayList<>();

        for (double i = 0; i <= intervals; i++) {
            result.add(
                Waypoint.fromPlannedPathState(
                    sample(
                        MathUtil.interpolate(
                            ti,
                            tf,
                            i * (1.0 / intervals)))));
        }

        return result;
    }

    public List<Waypoint> sampleTimeRange(double ti, double tf) {
        return sampleTimeRange(ti, tf, 10.0); // 10 intervals is accurate enough to match original path
    }

    private Pair<Trajectory.State, RotationSequence.State> _sample(double timeSeconds) {
        return
            Pair.of(
                this.driveTrajectory.sample(timeSeconds),
                this.rotationSequence.sample(timeSeconds));
    }

    public HolonomicState sample(double timeSeconds) {
        Pair<Trajectory.State, RotationSequence.State> sample = this._sample(timeSeconds);

        return
            new HolonomicState(
                sample.getFirst(),
                sample.getSecond());
    }

    public double getTotalTimeSeconds() {
        return getDriveTrajectory().getTotalTimeSeconds();
    }

    /**
     * Represents the state of the robot at a specific time in the trajectory.
     *
     * @param timeSeconds The time elapsed since the beginning of the trajectory.
     * @param velocityMetersPerSecond The speed at that point of the trajectory.
     * @param accelerationMetersPerSecondSq The acceleration at that point of the trajectory.
     * @param poseMeters The pose at that point of the trajectory.
     * @param curvatureRadPerMeter The curvature at that point of the trajectory.
     * @param holonomicAngle The direction the robot is facing.
     * @param angularVelocityRadiansPerSec The speed at which the robot is rotating.
     */
    public record HolonomicState(
        double timeSeconds,
        double velocityMetersPerSecond,
        double accelerationMetersPerSecondSq,
        Pose2d poseMeters,
        double curvatureRadPerMeter,
        Rotation2d holonomicAngle,
        double angularVelocityRadiansPerSec
    ) {
        public HolonomicState(Trajectory.State trajState, RotationSequence.State rotState) {
            this(
                trajState.timeSeconds,
                trajState.velocityMetersPerSecond,
                trajState.accelerationMetersPerSecondSq,
                trajState.poseMeters,
                trajState.curvatureRadPerMeter,
                rotState.position,
                rotState.velocityRadiansPerSec);
        }

        public HolonomicState withNewHolonomicAngle(Rotation2d newHolonomicAngle) {
            return
                new HolonomicState(
                    this.timeSeconds,
                    this.velocityMetersPerSecond,
                    this.accelerationMetersPerSecondSq,
                    this.poseMeters,
                    this.curvatureRadPerMeter,
                    newHolonomicAngle,
                    this.angularVelocityRadiansPerSec);
        }
    }
}