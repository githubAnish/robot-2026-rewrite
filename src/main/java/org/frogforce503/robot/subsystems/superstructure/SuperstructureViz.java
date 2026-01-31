package org.frogforce503.robot.subsystems.superstructure;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeomUtil.class})
public class SuperstructureViz {
    public SuperstructureViz() {}

    public void update(Pose3d drivePose3d, double turretAngleRad, double hoodAngleRad) {
        var turretPose =
            TurretConstants.robotToTurret
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
                .transformBy(TurretConstants.turretToCamera);

        Logger.recordOutput("SuperstructureViz/CameraPose", cameraPose);
    }
}