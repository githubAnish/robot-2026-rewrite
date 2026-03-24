package org.frogforce503.robot.subsystems.superstructure;

import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.vision.VisionConstants;
import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SuperstructureViz {
    private final Transform3d robotToHood = HoodConstants.robotToHood;
    private final Transform3d robotToIntakePivot = new Transform3d(0.28, 0.01, 0.206, Rotation3d.kZero);
    private final Transform3d robotToHopperExtender = new Transform3d(0.306, 0.01, 0.31, Rotation3d.kZero);
    private final Transform3d robotToClimber = new Transform3d(0.0625, -0.345, 0.185, Rotation3d.kZero);

    public void update(Pose3d drivePose3d, double hoodAngleRad, double intakePivotAngleRad, double climberHeightMeters) {
        // Calculate subsystem poses
        var hoodPose =
            Pose3d.kZero
                .plus(robotToHood)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -hoodAngleRad, Math.PI)));

        var intakePivotPose =
            Pose3d.kZero
                .plus(robotToIntakePivot)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -intakePivotAngleRad, 0)));

        var hopperExtenderPose =
            Pose3d.kZero
                .plus(robotToHopperExtender)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -MathUtil.clamp(intakePivotAngleRad, 0, Math.PI/2), 0.0)));

        var climberPose =
            Pose3d.kZero
                .plus(robotToClimber)
                .plus(new Transform3d(0, 0, climberHeightMeters, Rotation3d.kZero));

        Logger.recordOutput(
            "SuperstructureViz/Components",
            hoodPose, intakePivotPose, hopperExtenderPose, climberPose);

        // Calculate camera poses
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

        Logger.recordOutput("SuperstructureViz/LeftCameraPose", leftCameraPose);
        Logger.recordOutput("SuperstructureViz/RightCameraPose", rightCameraPose);
        Logger.recordOutput("SuperstructureViz/BackCameraPose", backCameraPose);
        Logger.recordOutput("SuperstructureViz/FuelCameraPose", fuelCameraPose);
    }
}