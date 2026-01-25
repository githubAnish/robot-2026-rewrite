package org.frogforce503.robot.subsystems.vision;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;
import org.frogforce503.robot.subsystems.vision.apriltagdetection.AprilTagIO;
import org.frogforce503.robot.subsystems.vision.apriltagdetection.AprilTagInputsAutoLogged;
import org.frogforce503.robot.subsystems.vision.objectdetection.ObjectDetectionIO;
import org.frogforce503.robot.subsystems.vision.objectdetection.ObjectDetectionInputsAutoLogged;
import org.frogforce503.lib.vision.apriltagdetection.PoseObservation;
import org.frogforce503.lib.vision.apriltagdetection.VisionMeasurement;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The subsystem that handles vision processing for AprilTag detection and object detection.
 */
public class Vision extends SubsystemBase {
    // Method that takes in a vision measurement as a parameter, fuses it with odometry, and returns void
    private Consumer<VisionMeasurement> visionConsumer;
    // Method that takes in no parameters and returns the robot's pose as a Pose2d
    private Supplier<Pose2d> robotPoseSupplier;

    // Maps camera names to their corresponding AprilTagIO instances.
    private EnumMap<CameraName, AprilTagIO> aprilTagIOMap = new EnumMap<>(CameraName.class);
    // Maps camera names to their corresponding AprilTagInputs instances.
    private EnumMap<CameraName, AprilTagInputsAutoLogged> aprilTagInputsMap = new EnumMap<>(CameraName.class);
    
    // The current goal for AprilTag detection, defaulting to global localization.
    private AprilTagGoal desiredAprilTagGoal = AprilTagGoal.GLOBAL_LOCALIZATION;


    // Maps camera names to their corresponding ObjectDetectionIO instances.
    private EnumMap<CameraName, ObjectDetectionIO> objectDetectionIOMap = new EnumMap<>(CameraName.class);
    // Maps camera names to their corresponding ObjectDetectionInputs instances.
    private EnumMap<CameraName, ObjectDetectionInputsAutoLogged> objectDetectionInputsMap = new EnumMap<>(CameraName.class);
    
    /**
     * @param visionConsumer The consumer that will fuse vision measurements into the robot pose
     * @param robotPoseSupplier The supplier that provides the robot's pose
     * @param aprilTagIOs Array of AprilTagIO for AprilTag detection
     * @param objectDetectionIOs Array of ObjectDetectionIO for object detection
     */
    public Vision(
        Consumer<VisionMeasurement> visionConsumer, 
        Supplier<Pose2d> robotPoseSupplier, 
        AprilTagIO[] aprilTagIOs, 
        ObjectDetectionIO[] objectDetectionIOs
    ) {
        this.visionConsumer = visionConsumer;
        this.robotPoseSupplier = robotPoseSupplier;

        //Populate maps
        for (int i = 0; i < aprilTagIOs.length; i++) {
            aprilTagIOMap.put(aprilTagIOs[i].getCameraName(), aprilTagIOs[i]);
            aprilTagInputsMap.put(aprilTagIOs[i].getCameraName(), new AprilTagInputsAutoLogged());

            logPoseObservation(aprilTagIOs[i], new PoseObservation());
        }

        for (int i = 0; i < objectDetectionIOs.length; i++) {
            objectDetectionIOMap.put(objectDetectionIOs[i].getCameraName(), objectDetectionIOs[i]);
            objectDetectionInputsMap.put(objectDetectionIOs[i].getCameraName(), new ObjectDetectionInputsAutoLogged());
        }
    }

    // TODO: Add logic to calculate the robot to camera offsets of turret cameras based on the turret angle

    @Override
    public void periodic() {
        /************************************ APRILTAG DETECTION LOGIC ************************************/
        EnumSet<AprilTagGoal> aprilTagGoalsRan = EnumSet.noneOf(AprilTagGoal.class); // Set to track which AprilTag goals have been run this periodic cycle.
        boolean anyAprilTagCamerasUsed = false; // Boolean to track if any AprilTag cameras were used for the current goal.

        AprilTagGoal currentAprilTagGoal = desiredAprilTagGoal; // The current AprilTag goal being processed.

        for (CameraName cameraName : aprilTagIOMap.keySet()) {
            AprilTagIO aprilTagIO = aprilTagIOMap.get(cameraName);
            AprilTagInputsAutoLogged aprilTagInputs = aprilTagInputsMap.get(cameraName);

            // Update AprilTag IO with the important information for accurate pose estimations.
            aprilTagIO.setRobotPose(robotPoseSupplier.get());

            // Update the inputs for the AprilTagIO and log them.
            aprilTagIO.updateInputs(aprilTagInputs);
            Logger.processInputs("Vision/AprilTag Detection/" + cameraName.name() + "/Inputs", aprilTagInputs);

            // Boolean representing if a VisionMeasurement from a camera's PoseObservation is accepted by the vision consumer.
            boolean visionMeasurementUsed = false;

            // Check if the camera is a potential camera to use for our current goal.
            if (currentAprilTagGoal.getCamerasToUse().contains(cameraName)) {
                Optional<VisionMeasurement> measurement = getVisionMeasurement(desiredAprilTagGoal, aprilTagIO);
                
                // If there is a vision measurement, it means the camera is outputting a pose observation reliable enough for the AprilTag goal.
                if (measurement.isPresent()) {
                    anyAprilTagCamerasUsed = true; // Set the boolean to true since we used a vision measurement.

                    visionConsumer.accept(measurement.get()); // Pass the measurement to the vision consumer so it can be fused with odometry.
                    visionMeasurementUsed = true; // Set the boolean to true since we used a vision measurement.
                }
            } else {
                // If the camera is not being used for the current goal, log default values for the Pose Observation
                logPoseObservation(aprilTagIO, new PoseObservation());
            }

            Logger.recordOutput(
                "Vision/AprilTag Detection/" + cameraName.name() + "/Is Vision Measurement Used", 
                visionMeasurementUsed
            );

            aprilTagGoalsRan.add(currentAprilTagGoal); // Add the current goal to the set of goals that have been run.
        }

        /*
        If no vision measurements were accepted, the current goal has a backup goal, and the backup goal was never run before, try the backup goal.
        Loop in case backup goals have backup goals – effort to ensure we are always using vision measurements.
        Checks if the backup goal has been run before to avoid infinite loops in case of a cycle in the backup goals.
        */
        while (!anyAprilTagCamerasUsed && currentAprilTagGoal.getBackupGoal().isPresent() && !aprilTagGoalsRan.contains(currentAprilTagGoal.getBackupGoal().get())) {
            currentAprilTagGoal = currentAprilTagGoal.getBackupGoal().get();

            for (CameraName cameraName : currentAprilTagGoal.getCamerasToUse()) {
                AprilTagIO aprilTagIO = aprilTagIOMap.get(cameraName);
                AprilTagInputsAutoLogged aprilTagInputs = aprilTagInputsMap.get(cameraName);

                if (aprilTagIO == null || aprilTagInputs == null) {
                    continue; // Skip if the camera is not an IO on the current robot, hashmaps will return null value if key isn't in the map.
                }
    
                Optional<VisionMeasurement> measurement = getVisionMeasurement(currentAprilTagGoal, aprilTagIO);
                
                // If there is a vision measurement, it means the camera is outputting a pose observation reliable enough for the AprilTag goal.
                if (measurement.isPresent()) {
                    anyAprilTagCamerasUsed = true; // Set the boolean to true since we used a vision measurement.

                    visionConsumer.accept(measurement.get()); // Pass the measurement to the vision consumer so it can be fused with odometry.
                    
                    Logger.recordOutput("Vision/AprilTag Detection/" + cameraName.name() + "/Is Vision Measurement Used", true);
                }
            }

            aprilTagGoalsRan.add(currentAprilTagGoal); // Add the current goal to the set of goals that have been run.
        }

        // Log desired goal and current goal
        Logger.recordOutput("Vision/AprilTag Detection/DesiredGoal", desiredAprilTagGoal);
        Logger.recordOutput("Vision/AprilTag Detection/CurrentGoal", currentAprilTagGoal);

        
        /************************************ OBJECT DETECTION LOGIC ************************************/
        for (CameraName cameraName : objectDetectionIOMap.keySet()) {
            ObjectDetectionIO objectDetectionIO = objectDetectionIOMap.get(cameraName);
            ObjectDetectionInputsAutoLogged objectDetectionInputs = objectDetectionInputsMap.get(cameraName);

            objectDetectionIO.updateInputs(objectDetectionInputs);
            Logger.processInputs("Vision/Object Detection/" + cameraName.name() + "/Inputs", objectDetectionInputs);
        }
    }

    /************************************ PUBLIC METHODS ************************************/

    public void setDesiredAprilTagGoal(AprilTagGoal goal) {
        this.desiredAprilTagGoal = goal;
    }


    /************************************ PRIVATE METHODS ************************************/

    /**
     * Gets a VisionMeasurement from the AprilTagIO based on the current goal.
     * The camera is configured based on the goal's camera configuration, and the outputted pose observation is checked against the goal's camera filter.
     * If the camera has a vision measurement that meets the goal's criteria, a VisionMeasurement is created and returned.
     * 
     * @param goal the AprilTag goal being used to determine the camera configuration and filtering criteria
     * @param aprilTagIO the AprilTagIO instance for the camera being used
     * @return an Optional<VisionMeasurement> which is empty if no valid measurement is found, or contains a VisionMeasurement if a valid one is found.
     */
    private Optional<VisionMeasurement> getVisionMeasurement(AprilTagGoal goal, AprilTagIO aprilTagIO) {
        Optional<VisionMeasurement> measurement = Optional.empty();

        goal.getCameraConfiguration().accept(aprilTagIO); // Configure pose estimation parameters of the AprilTagIO for the AprilTagGoal
        PoseObservation poseObservation = aprilTagIO.estimateRobotPose(); // Estimate the robot's pose using the AprilTagIO.
        logPoseObservation(aprilTagIO, poseObservation);

        if (poseObservation.isReal()) { // Check if the pose observation is actually real
            // Check if the camera should be used for localization.
            if (goal.getCameraFilter().test(poseObservation)) {
                Matrix<N3, N1> standardDeviations = goal.getStandardDeviationCalculator().apply(poseObservation);

                // Create a VisionMeasurement and pass it to the vision consumer.
                measurement = Optional.of(
                    new VisionMeasurement(
                        poseObservation.timestamp(),
                        poseObservation.robotPose().toPose2d(),
                        standardDeviations
                    )
                );
            }
        }

        return measurement;
    }

    /**
     * Pose observations contain an array of TrackedAprilTags that aren't automatically logged by the Logger.
     * This method logs the pose observation and the array of TrackedAprilTags it used to estimate the robot's pose.
     * 
     * @param aprilTagIO AprilTagIO instance of the camera outputting the pose observation
     * @param poseObservation The outputted pose observation to log
     */
    private void logPoseObservation(AprilTagIO aprilTagIO, PoseObservation poseObservation) {
        Logger.recordOutput("Vision/AprilTag Detection/" + aprilTagIO.getCameraName().name() + "/Pose Observation", poseObservation);
        Logger.recordOutput("Vision/AprilTag Detection/" + aprilTagIO.getCameraName().name() + "/Pose Observation/Used April Tags", poseObservation.usedAprilTags());
    }
}
