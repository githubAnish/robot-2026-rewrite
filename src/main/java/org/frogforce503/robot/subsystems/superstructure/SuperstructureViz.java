package org.frogforce503.robot.subsystems.superstructure;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.frogforce503.robot.subsystems.vision.VisionConstants;
import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SuperstructureViz {
    // Constants
    private final Transform3d robotToTurret =
        TurretConstants.robotToTurret
            .plus(new Transform3d(-0.0076, -0.0041, 0.041686, Rotation3d.kZero)); // Offset for viz

    private final Transform3d turretToHood = HoodConstants.turretToHood;
    private final Transform3d robotToIntakePivot = new Transform3d(0.28, 0.01, 0.206, Rotation3d.kZero);
    private final Transform3d robotToHopperExtender = new Transform3d(0.306, 0.01, 0.31, Rotation3d.kZero);
    private final Transform3d robotToClimberDeployMainPivot = new Transform3d(0.0625, -0.345, 0.185, Rotation3d.kZero);
    private final Transform3d climberDeployMainPivotToSecondaryPivot = new Transform3d(0.0, 0.075, -0.1, Rotation3d.kZero); // due to 4 bar for climber

    public SuperstructureViz() {}

    public void update(Pose3d drivePose3d, double turretAngleRad, double hoodAngleRad, double intakePivotAngleRad, double climberDeployAngleRad) {
        // Calculate subsystem poses
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
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -intakePivotAngleRad, 0)));

        var hopperExtenderPose =
            Pose3d.kZero
                .plus(robotToHopperExtender)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -Math.PI/2, 0.0)));

        var climberDeployFourBarPose =
            Pose3d.kZero
                .plus(robotToClimberDeployMainPivot)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(climberDeployAngleRad, 0.0, 0.0)));

        var climberHookPose =
            climberDeployFourBarPose
                .plus(climberDeployMainPivotToSecondaryPivot)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(-climberDeployAngleRad, 0.0, 0.0)));

        Logger.recordOutput(
            "SuperstructureViz/Components",
            turretPose, hoodPose, intakePivotPose, hopperExtenderPose, climberDeployFourBarPose, climberHookPose);

        // Calculate camera poses
        var turretCameraPose =
            drivePose3d
                .plus(GeomUtil.toTransform3d(turretPose))
                .plus(VisionConstants.turretToTurretCameraOffsets.get(CameraName.TURRET_CAMERA));

        var leftCameraPose =
            drivePose3d
                .plus(VisionConstants.robotToFixedCameraOffsets.get(CameraName.LEFT_CAMERA));

        var rightCameraPose =
            drivePose3d
                .plus(VisionConstants.robotToFixedCameraOffsets.get(CameraName.RIGHT_CAMERA));

        var backCameraPose =
            drivePose3d
                .plus(VisionConstants.robotToFixedCameraOffsets.get(CameraName.BACK_CAMERA));

        var fuelCameraPose =
            drivePose3d
                .plus(VisionConstants.robotToFixedCameraOffsets.get(CameraName.FUEL_CAMERA));

        Logger.recordOutput("SuperstructureViz/TurretCameraPose", turretCameraPose);
        Logger.recordOutput("SuperstructureViz/LeftCameraPose", leftCameraPose);
        Logger.recordOutput("SuperstructureViz/RightCameraPose", rightCameraPose);
        Logger.recordOutput("SuperstructureViz/BackCameraPose", backCameraPose);
        Logger.recordOutput("SuperstructureViz/FuelCameraPose", fuelCameraPose);
    }
}