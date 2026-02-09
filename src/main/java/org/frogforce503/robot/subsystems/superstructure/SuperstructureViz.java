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
    // Constants
    private final Transform3d robotToTurret = TurretConstants.robotToTurret;
    private final Transform3d turretToHood = new Transform3d(0.104, 0, 0.09, Rotation3d.kZero);
    private final Transform3d robotToIntakePivot = new Transform3d(0.28, 0.01, 0.206, Rotation3d.kZero);
    private final Transform3d robotToHopperExtender = new Transform3d(0.306, 0.01, 0.31, Rotation3d.kZero);

    public SuperstructureViz() {}

    public void update(Pose3d drivePose3d, double turretAngleRad, double hoodAngleRad, double intakePivotAngleRad) {
        updateFF(drivePose3d, turretAngleRad, hoodAngleRad, intakePivotAngleRad);
        // update6328(drivePose3d, turretAngleRad, hoodAngleRad);
    }

    public void updateFF(Pose3d drivePose3d, double turretAngleRad, double hoodAngleRad, double intakePivotAngleRad) {
        var turretPose =
            Pose3d.kZero
                .plus(robotToTurret)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, 0.0, turretAngleRad)));

        var hoodPose =
            turretPose
                .plus(turretToHood)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -hoodAngleRad, Math.PI)));

        var intakePivotPose =
            Pose3d.kZero
                .plus(robotToIntakePivot)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -intakePivotAngleRad + Math.PI/2, 0)));

        var hopperExtenderPose =
            Pose3d.kZero
                .plus(robotToHopperExtender)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, 0.0, 0.0)));

        Logger.recordOutput("SuperstructureViz/Components", turretPose, hoodPose, intakePivotPose, hopperExtenderPose);
    }

    public void update6328(Pose3d drivePose3d, double turretAngleRad, double hoodAngleRad) {
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