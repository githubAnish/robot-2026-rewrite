package org.frogforce503.lib.vision.apriltag_detection;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Record representing a vision measurement from an AprilTagIO that is fused with odometry.
 *
 * @param timestamp The time at which the measurement was taken.
 * @param pose The estimated pose of the detected object.
 * @param standardDeviations The standard deviations of the pose measurement's x-coordinate and y-coordinate in meters along with its heading in radians.
 */
public record VisionMeasurement(
    double timestamp,
    Pose2d pose,
    Matrix<N3, N1> standardDeviations
) {}