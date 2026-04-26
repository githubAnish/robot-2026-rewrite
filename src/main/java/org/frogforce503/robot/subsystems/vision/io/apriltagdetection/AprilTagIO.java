package org.frogforce503.robot.subsystems.vision.io.apriltagdetection;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface AprilTagIO {
    @AutoLog
    class AprilTagIOInputs {
        public double timestamp;
        public Pose3d estimatedRobotPose;
        public Matrix<N3, N1> estimatedStdDevs;
    }

    default void updateInputs(AprilTagIOInputs inputs) {}
}