package org.frogforce503.robot.subsystems.superstructure;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class SuperstructureViz {
    // Requirements
    private final Superstructure superstructure;
    private final Supplier<Pose2d> robotPoseSupplier;

    // 2D Viz Constants

    // 2D Viz Mechanisms
    private final LoggedMechanism2d mechanism2d =
        new LoggedMechanism2d(
            Units.inchesToMeters(50),
            Units.inchesToMeters(80));

    // 3D Viz Constants

    // 3D Viz Poses
    
    public SuperstructureViz(Superstructure superstructure, Supplier<Pose2d> robotPoseSupplier) {
        this.superstructure = superstructure;
        this.robotPoseSupplier = robotPoseSupplier;

        // Setup 2D Viz
    }

    public void update(double turretAngleRad, double hoodAngleRad) {
        Pose3d drivePose3d = new Pose3d(robotPoseSupplier.get());

        update2dViz(drivePose3d);
        update3dViz(drivePose3d, turretAngleRad, hoodAngleRad);
    }

    private void update2dViz(Pose3d drivePose3d) {
        // Update Ligaments

        Logger.recordOutput("SuperstructureViz/2D", mechanism2d);
    }

    private void update3dViz(Pose3d drivePose3d, double turretAngleRad, double hoodAngleRad) {
        var turretPose =
            new Pose3d(-0.197, 0.0, 0.44, new Rotation3d(0.0, 0.0, turretAngleRad));

        var hoodPose =
            turretPose.transformBy(
                new Transform3d(
                    0.105, 0.0, 0.092, new Rotation3d(0.0, -hoodAngleRad, Math.PI)));

        Logger.recordOutput("SuperstructureViz/3D/Components", turretPose, hoodPose);
    }
}