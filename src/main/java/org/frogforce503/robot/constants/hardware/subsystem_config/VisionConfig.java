package org.frogforce503.robot.constants.hardware.subsystem_config;

import java.util.EnumMap;

import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;

import edu.wpi.first.math.geometry.Transform3d;

public record VisionConfig(
    EnumMap<CameraName, Transform3d> robotToFixedCameraOffsets,
    EnumMap<CameraName, Transform3d> turretToTurretCameraOffsets,
    Transform3d robotToTurretBaseOffset
) {}