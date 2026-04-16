package org.frogforce503.robot.subsystems.vision;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.frogforce503.robot.subsystems.vision.VisionConstants.CameraName;
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagInputsAutoLogged;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIO;
import org.frogforce503.lib.vision.apriltagdetection.PoseObservation;
import org.frogforce503.lib.vision.apriltagdetection.VisionMeasurement;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vision extends SubsystemBase {
    private final EnumMap<CameraName, AprilTagIO> aprilTagIOMap = new EnumMap<>(CameraName.class);
    private final EnumMap<CameraName, AprilTagInputsAutoLogged> aprilTagInputsMap = new EnumMap<>(CameraName.class);

    private final Consumer<VisionMeasurement> visionEstimateConsumer;
    private final Supplier<Pose2d> robotPoseSupplier;
    
    @Setter
    private AprilTagGoal desiredAprilTagGoal = AprilTagGoal.GLOBAL_LOCALIZATION;

    // Performance tuning
    private EnumMap<CameraName, VisionConstants.VisionLogPaths> pathMap = new EnumMap<>(CameraName.class);
    private int poseObservationLoggingCounter = 0;
    
    public Vision(Consumer<VisionMeasurement> visionConsumer, Supplier<Pose2d> robotPoseSupplier, AprilTagIO[] aprilTagIOs) {
        this.visionEstimateConsumer = visionConsumer;
        this.robotPoseSupplier = robotPoseSupplier;
        
        for (int i = 0; i < aprilTagIOs.length; i++) {
            CameraName name = aprilTagIOs[i].getCameraName();

            aprilTagIOMap.put(name, aprilTagIOs[i]);
            aprilTagInputsMap.put(name, new AprilTagInputsAutoLogged());

            pathMap.put(name, new VisionConstants.VisionLogPaths(name, true));

            logPoseObservation(aprilTagIOs[i], PoseObservation.kZero); // for warmup
        }
    }

    @Override
    public void periodic() {
        boolean shouldLogPoseObservation = poseObservationLoggingCounter == 0;

        EnumSet<AprilTagGoal> aprilTagGoalsRan = EnumSet.noneOf(AprilTagGoal.class);
        boolean anyAprilTagCamerasUsed = false;

        AprilTagGoal currentAprilTagGoal = desiredAprilTagGoal;

        for (CameraName cameraName : aprilTagIOMap.keySet()) {
            VisionConstants.VisionLogPaths paths = pathMap.get(cameraName);

            AprilTagIO aprilTagIO = aprilTagIOMap.get(cameraName);
            AprilTagInputsAutoLogged aprilTagInputs = aprilTagInputsMap.get(cameraName);

            aprilTagIO.setRobotPose(robotPoseSupplier.get());
            aprilTagIO.updateInputs(aprilTagInputs);
            
            Logger.processInputs(paths.inputs, aprilTagInputs);

            // Get vision measurement
            boolean visionMeasurementUsed = false;

            if (currentAprilTagGoal.getCamerasToUse().contains(cameraName)) {
                Optional<VisionMeasurement> measurement =
                    getVisionMeasurement(desiredAprilTagGoal, aprilTagIO, shouldLogPoseObservation);
                
                if (measurement.isPresent()) {
                    visionEstimateConsumer.accept(measurement.get());
                    anyAprilTagCamerasUsed = true;
                    visionMeasurementUsed = true;
                }
            } 

            Logger.recordOutput(paths.isUsed, visionMeasurementUsed);

            aprilTagGoalsRan.add(currentAprilTagGoal);
        }

        /*
         * If no vision measurements were accepted, the current goal has a backup goal, and the backup goal was never run before, try the backup goal.
         * Loop in case backup goals have backup goals – effort to ensure we are always using vision measurements.
         * Checks if the backup goal has been run before to avoid infinite loops in case of a cycle in the backup goals.
        */
        while (
            !anyAprilTagCamerasUsed &&
            currentAprilTagGoal.getBackupGoal().isPresent() &&
            !aprilTagGoalsRan.contains(currentAprilTagGoal.getBackupGoal().get())
        ) {
            currentAprilTagGoal = currentAprilTagGoal.getBackupGoal().get();

            for (CameraName cameraName : currentAprilTagGoal.getCamerasToUse()) {
                AprilTagIO aprilTagIO = aprilTagIOMap.get(cameraName);
                AprilTagInputsAutoLogged aprilTagInputs = aprilTagInputsMap.get(cameraName);

                if (aprilTagIO == null || aprilTagInputs == null) {
                    continue; // Skip if camera is not an IO on current robot, HashMaps will return null if key not in map
                }
    
                Optional<VisionMeasurement> measurement =
                    getVisionMeasurement(currentAprilTagGoal, aprilTagIO, shouldLogPoseObservation);
                
                if (measurement.isPresent()) {
                    visionEstimateConsumer.accept(measurement.get());
                    anyAprilTagCamerasUsed = true;
                }
            }

            aprilTagGoalsRan.add(currentAprilTagGoal);
        }

        Logger.recordOutput("Vision/AprilTag Detection/DesiredGoal", desiredAprilTagGoal);
        Logger.recordOutput("Vision/AprilTag Detection/CurrentGoal", currentAprilTagGoal);

        poseObservationLoggingCounter = (poseObservationLoggingCounter + 1) % VisionConstants.POSE_OBSERVATION_LOGGING_FREQUENCY;
    }

    /**
     * Gets a VisionMeasurement from the AprilTagIO based on the current goal.
     * The camera is configured based on the goal's camera configuration, and the outputted pose observation is checked against the goal's camera filter.
     * If the camera has a vision measurement that meets the goal's criteria, a VisionMeasurement is created and returned.
     */
    private Optional<VisionMeasurement> getVisionMeasurement(AprilTagGoal goal, AprilTagIO aprilTagIO, boolean shouldLog) {
        Optional<VisionMeasurement> measurement = Optional.empty();

        goal.getCameraConfiguration().accept(aprilTagIO); // Configure pose estimation parameters of the AprilTagIO for the AprilTagGoal
        PoseObservation poseObservation = aprilTagIO.estimateRobotPose();

        if (shouldLog) {
            logPoseObservation(aprilTagIO, poseObservation);
        }

        if (poseObservation.isReal()) {
            // Check if camera should be used for localization
            if (goal.getCameraFilter().test(poseObservation)) {
                Matrix<N3, N1> standardDeviations =
                    goal
                        .getStandardDeviationCalculator()
                        .apply(poseObservation);

                measurement =
                    Optional.of(
                        new VisionMeasurement(
                            poseObservation.timestamp(),
                            poseObservation.robotPose().toPose2d(),
                            standardDeviations));
            }
        }

        return measurement;
    }

    /**
     * Pose observations contain an array of TrackedAprilTags that aren't automatically logged by the Logger.
     * This method logs the pose observation and the array of TrackedAprilTags it used to estimate the robot's pose.
     */
    private void logPoseObservation(AprilTagIO aprilTagIO, PoseObservation poseObservation) {
        VisionConstants.VisionLogPaths paths = pathMap.get(aprilTagIO.getCameraName());
        
        Logger.recordOutput(paths.robotPose, poseObservation.robotPose().toPose2d());

        // Avoid stream & use simple loop to reduce GC overhead
        var tags = poseObservation.usedAprilTags();
        int[] ids = new int[tags.length];

        for (int i = 0; i < tags.length; i++) {
            ids[i] = tags[i].tagID();
        }

        Logger.recordOutput(paths.obsType, poseObservation.poseObservationType());
        Logger.recordOutput(paths.tags, ids); // AdvantageKit handles int[] natively and faster than Strings
    }
}
