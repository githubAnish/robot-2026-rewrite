package org.frogforce503.robot.viz;

import org.frogforce503.robot.FieldConstants;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.Getter;

public class VisionSimulator {
    @Getter private final VisionSystemSim aprilTagDetectionSimulator;

    public VisionSimulator() {
        aprilTagDetectionSimulator = new VisionSystemSim("AprilTagSimulator");
        aprilTagDetectionSimulator.addAprilTags(FieldConstants.aprilTagFieldLayout);
    }

    public void update(Pose2d robotPose) {
        aprilTagDetectionSimulator.update(robotPose);
    }
}
