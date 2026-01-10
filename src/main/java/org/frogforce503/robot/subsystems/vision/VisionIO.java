package org.frogforce503.robot.subsystems.vision;

import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Interface to define boilerplate I/O functions of any vision system. 
 * Should be extended by other I/O interfaces, not implemented.
 */
public interface VisionIO {
    /**
     * Gets the name of the camera.
     * 
     * @return The CameraName enum representing the camera's name
     */
    CameraName getCameraName();

    /**
     * Sets the offset from the robot's origin to the camera's origin.
     * The robot origin is the center of the robot on the floor, the camera origin is the center of the lens.
     * 
     * @return The transform3d representing the offset using the robot coordinate system.
     */
    Transform3d getRobotToCameraOffset();

    /**
     * Sets the offset from the robot's origin to the camera's origin.
     * The robot origin is the center of the robot on the floor, the camera origin is the center of the lens.
     * 
     * @param robotToCameraOffset The transform3d representing the offset using the robot coordinate system.
     */
    void setRobotToCameraOffset(Transform3d robotToCameraOffset);


    /**
     * Gets the pipeline index.
     * 
     * @return The index of the current pipeline or -1 if the camera is not connected
     */
    int getPipeline();

    /**
     * Sets the pipeline index.
     * 
     * @param pipelineIndex The index of the desired pipeline, cannot be negative
     */
    void setPipeline(int pipelineIndex);
}
