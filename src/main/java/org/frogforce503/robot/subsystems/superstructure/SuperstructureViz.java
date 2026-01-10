package org.frogforce503.robot.subsystems.superstructure;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

    public void update() {
        Pose3d drivePose3d = new Pose3d(robotPoseSupplier.get());

        update2dViz(drivePose3d);
        update3dViz(drivePose3d);
    }

    private void update2dViz(Pose3d drivePose3d) {
        // Update Ligaments

        Logger.recordOutput("SuperstructureViz/2D", mechanism2d);
    }

    private void update3dViz(Pose3d drivePose3d) {

    }
}