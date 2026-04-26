package org.frogforce503.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Camera configurations
    public static final CameraConfig leftCameraConfig =
        new CameraConfig("LeftCamera", Transform3d.kZero);

    public static final CameraConfig rightCameraConfig =
        new CameraConfig("RightCamera", Transform3d.kZero);

    public static final CameraConfig backCameraConfig =
        new CameraConfig("BackCamera", Transform3d.kZero);

    public static final CameraConfig backLeftCameraConfig =
        new CameraConfig("BackLeftCamera", Transform3d.kZero);

    public record CameraConfig(
        String name,
        Transform3d robotToCamera) {}

    @FunctionalInterface
    public interface VisionEstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
    }
}