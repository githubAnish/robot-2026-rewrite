package org.frogforce503.robot.subsystems.superstructure;

import java.util.function.Supplier;

import org.frogforce503.lib.math.GeomUtil;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeomUtil.class})
public class SuperstructureViz {
    // Requirements
    private final Supplier<Pose2d> robotPoseSupplier;

    // Constants
    public static Transform3d robotToTurret = new Transform3d(-0.19685, 0.0, 0.44, Rotation3d.kZero);
    public static Transform3d turretToCamera =
        new Transform3d(
            -0.1314196, 0.0, 0.2770674, new Rotation3d(0.0, Units.degreesToRadians(-22.5), 0.0));
    
    public SuperstructureViz(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    public void update(double turretAngleRad, double hoodAngleRad) {
        Pose3d drivePose3d = new Pose3d(robotPoseSupplier.get());

        update3dViz(drivePose3d, turretAngleRad, hoodAngleRad);
    }

    private void update3dViz(Pose3d drivePose3d, double turretAngleRad, double hoodAngleRad) {
        var turretPose =
            robotToTurret
                .toPose3d()
                .transformBy(
                    new Transform3d(
                        Translation3d.kZero, new Rotation3d(0.0, 0.0, turretAngleRad)));

        var hoodPose =
            turretPose.transformBy(
                new Transform3d(
                    0.105, 0.0, 0.092, new Rotation3d(0.0, -hoodAngleRad, Math.PI)));

        Logger.recordOutput("SuperstructureViz/Components", turretPose, hoodPose);

        var cameraPose =
            drivePose3d
                .transformBy(turretPose.toTransform3d())
                .transformBy(turretToCamera);

        Logger.recordOutput("SuperstructureViz/CameraPose", cameraPose);
    }
}