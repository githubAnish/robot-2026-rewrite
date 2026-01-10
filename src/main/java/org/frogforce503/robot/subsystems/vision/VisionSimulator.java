package org.frogforce503.robot.subsystems.vision;

import org.frogforce503.robot.Constants;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import lombok.Getter;

/**
 * Wrapper class for multiple PhotonVision VisionSystemSims. 
 * Manages simulation worlds for object detection and AprilTag detection cameras.
 */
public class VisionSimulator {
    //AprilTag Detection
    private VisionSystemSim aprilTagDetectionSimulator;
    @Getter private AprilTagFieldLayout aprilTagFieldLayout;

    //Object Detection
    private VisionSystemSim objectDetectionSimulator;

    /**
     * @param aprilTagFieldLayout The AprilTagFieldLayout to use for AprilTag detection simulation.
     */
    public VisionSimulator(AprilTagFieldLayout aprilTagFieldLayout) {
        this.aprilTagFieldLayout = aprilTagFieldLayout;
    }

    public VisionSimulator() {
        this(Constants.fieldVenue.getAprilTagFieldLayout());
    }

    /**
     * Updates the simulation state with the robot's current pose.
     * This method should be called periodically to ensure the simulation reflects the robot's position.
     * 
     * @param robotPose The current pose of the robot in the simulation.
     */
    public void update(Pose2d robotPose) {
        if (aprilTagDetectionSimulator != null) {
            aprilTagDetectionSimulator.update(robotPose);
        }

        if (objectDetectionSimulator != null) {
            objectDetectionSimulator.update(robotPose);
        }
    }

    /************************************ APRILTAG DETECTION LOGIC ************************************/

    /**
     * Gets the VisionSystemSim that simulates the robot's AprilTag detection system.
     * Utilizes lazy initialization to create the simulator only when it is first requested.
     * 
     * @return The VisionSystemSim instance used for AprilTag detection.
     */
    public VisionSystemSim getAprilTagDetectionSimulator() {
        if (aprilTagDetectionSimulator == null) {
            aprilTagDetectionSimulator = new VisionSystemSim("AprilTagSimulator");

            aprilTagDetectionSimulator.addAprilTags(aprilTagFieldLayout);
        }

        return aprilTagDetectionSimulator;
    }

    /************************************ OBJECT DETECTION LOGIC ************************************/

    /**
     * Gets the VisionSystemSim that simulates the robot's object detection system.
     * Utilizes lazy initialization to create the simulator only when it is first requested.
     * 
     * @return The VisionSystemSim instance used for object detection.
     */
    public VisionSystemSim getObjectDetectionSimulator() {
        if (objectDetectionSimulator == null) {
            objectDetectionSimulator = new VisionSystemSim("ObjectDetectionSimulator");
        }

        return objectDetectionSimulator;
    }

    /**
     * Clears all objects from the object detection simulator.
     * This method is useful for resetting the simulation environment.
     */
    public void clearObjects() {
        if (objectDetectionSimulator != null) {
            objectDetectionSimulator.clearVisionTargets();
        }
    }
}
