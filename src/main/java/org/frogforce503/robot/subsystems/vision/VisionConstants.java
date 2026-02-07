package org.frogforce503.robot.subsystems.vision;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

import org.frogforce503.lib.vision.VisionUtils;
import org.frogforce503.lib.vision.apriltagdetection.PoseObservation;
import org.frogforce503.lib.vision.apriltagdetection.PoseObservationType;
import org.frogforce503.lib.vision.apriltagdetection.TrackedAprilTag;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIO;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIOPhotonSim;
import org.frogforce503.robot.subsystems.vision.io.apriltagdetection.AprilTagIOPhotonVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

/**
 * Important constants used by the Vision subsystem such as the different camera names and sets of AprilTag IDs.
 */
public class VisionConstants {
    // Hardware / Configuration
    public static EnumMap<CameraName, Transform3d> robotToFixedCameraOffsets;
    public static EnumMap<CameraName, Transform3d> turretToTurretCameraOffsets;

    static {
        turretToTurretCameraOffsets.put(
            CameraName.FAR_TURRET_CAMERA,
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0)
            )
        );

        turretToTurretCameraOffsets.put(
            CameraName.CLOSE_TURRET_CAMERA,
            new Transform3d(
                new Translation3d(Units.inchesToMeters(2.321), Units.inchesToMeters(-.875), Units.inchesToMeters(1.141)),
                new Rotation3d(0, Units.degreesToRadians(30), 0)
            )
        );

        robotToFixedCameraOffsets.put(
            CameraName.INTAKE_LEFT_CAMERA,
            new Transform3d(
                new Translation3d(Units.inchesToMeters(1.288), Units.inchesToMeters(11.525), Units.inchesToMeters(19.956)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(-15))  
            )
        );

        robotToFixedCameraOffsets.put(
            CameraName.INTAKE_RIGHT_CAMERA,
            new Transform3d(
                new Translation3d(Units.inchesToMeters(1.288), Units.inchesToMeters(-11.525), Units.inchesToMeters(31.721)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(10))
            )
        );

        robotToFixedCameraOffsets.put(
            CameraName.BACK_CAMERA,
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0)
            )
        );

        robotToFixedCameraOffsets.put(
            CameraName.FUEL_CAMERA,
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0)
            )
        );
    }

    /**
     * Cameras on robots are configured with a name.
     * Every camera on the robot must have a name from this enum.
     * This enum is used to identify specific cameras on a robot for any use case.
     */
    public enum CameraName {
        // AprilTag Detection Cameras
        CLOSE_TURRET_CAMERA,
        FAR_TURRET_CAMERA,
        INTAKE_LEFT_CAMERA,
        INTAKE_RIGHT_CAMERA,
        BACK_CAMERA,

        // Object Detection Cameras
        FUEL_CAMERA
    }

    public static final Set<Integer> RED_TOWER_TAGS = Set.of(15, 16);
    public static final Set<Integer> RED_HUB_TAGS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    public static final Set<Integer> RED_OUTPOST_TAGS = Set.of(13, 14);
    public static final Set<Integer> RED_TRENCH_TAGS = Set.of(1, 6, 7, 12);

    public static final Set<Integer> BLUE_TOWER_TAGS = Set.of(31, 32);
    public static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
    public static final Set<Integer> BLUE_OUTPOST_TAGS = Set.of(29, 30);
    public static final Set<Integer> BLUE_TRENCH_TAGS = Set.of(17, 22, 23, 28);

    public static final Matrix<N3, N1> DEFAULT_STANDARD_DEVIATIONS = VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30));

    /**
     * Enum representing different goals for robot localization with AprilTags.
     * Each goal specifies the cameras to use, a filter to check if a vision measurement is reliable, a configuration for a camera, a standard deviation calculator for fusing vision measurements, and an optional backup goal.
     */
    public enum AprilTagGoal {
        /**
         * Uses every AprilTag camera available on the robot for localization.
         * Should not be used for precise tasks such as automated alignment.
         */
        GLOBAL_LOCALIZATION(
            EnumSet.of(
                CameraName.INTAKE_LEFT_CAMERA, 
                CameraName.INTAKE_RIGHT_CAMERA, 
                CameraName.CLOSE_TURRET_CAMERA, 
                CameraName.FAR_TURRET_CAMERA, 
                CameraName.BACK_CAMERA
            ),

            poseObservation -> {
                TrackedAprilTag[] tags = poseObservation.usedAprilTags();
                
                double ambiguity = VisionUtils.getLowestAmbiguity(tags);
                double distance = VisionUtils.getLowestDistanceToCamera(tags);

                double maxAmbiguity = tags.length > 1 ? 0.15 : 0.10;
                double maxDistance = tags.length > 1 ? Units.feetToMeters(15) : Units.feetToMeters(10);

                return ambiguity <= maxAmbiguity && distance <= maxDistance;
            },

            aprilTagIO -> {
                if (aprilTagIO instanceof AprilTagIOPhotonVision || aprilTagIO instanceof AprilTagIOPhotonSim) {
                    aprilTagIO.setPoseObservationType(PoseObservationType.MULTI_TAG_PNP_ON_COPROCESSOR);
                    aprilTagIO.setSecondaryPoseObservationType(PoseObservationType.LOWEST_AMBIGUITY);
                } 
                aprilTagIO.setIgnoredAprilTags(new HashSet<Integer>());
            },

            (poseObservation) -> VisionConstants.DEFAULT_STANDARD_DEVIATIONS,

            Optional.empty()
        ),



        /**
         * Uses backup AprilTag cameras for aiming tasks when primary turret cameras are unreliable.
         * Suitable for fallback scenarios during aiming.
         */
        STANDARD_HUB_AIM(
            EnumSet.of(
                CameraName.CLOSE_TURRET_CAMERA, 
                CameraName.FAR_TURRET_CAMERA,
                CameraName.INTAKE_LEFT_CAMERA,
                CameraName.INTAKE_RIGHT_CAMERA,
                CameraName.BACK_CAMERA
            ),
            poseObservation -> {
                TrackedAprilTag[] tags = poseObservation.usedAprilTags();
                
                double ambiguity = VisionUtils.getLowestAmbiguity(tags);
                double distance = VisionUtils.getLowestDistanceToCamera(tags);

                double maxAmbiguity = tags.length > 1 ? 0.10 : 0.07;
                double maxDistance = tags.length > 1 ? Units.feetToMeters(21.5) : Units.feetToMeters(15); 

                return ambiguity <= maxAmbiguity && distance <= maxDistance;
            },
             aprilTagIO -> {
                if (aprilTagIO instanceof AprilTagIOPhotonVision || aprilTagIO instanceof AprilTagIOPhotonSim) {
                    aprilTagIO.setPoseObservationType(PoseObservationType.MULTI_TAG_PNP_ON_COPROCESSOR);
                    aprilTagIO.setSecondaryPoseObservationType(PoseObservationType.LOWEST_AMBIGUITY);
                } 

                Set<Integer> ignoredTags = new HashSet<Integer>();
                ignoredTags.addAll(RED_TRENCH_TAGS);
                ignoredTags.addAll(BLUE_TRENCH_TAGS);
                ignoredTags.addAll(RED_OUTPOST_TAGS);
                ignoredTags.addAll(BLUE_OUTPOST_TAGS);
                ignoredTags.addAll(RED_TOWER_TAGS);
                ignoredTags.addAll(BLUE_TOWER_TAGS);
                
                if (FieldConstants.isRed()) {
                    ignoredTags.addAll(BLUE_HUB_TAGS);
                } else {
                    ignoredTags.addAll(RED_HUB_TAGS);
                }

                aprilTagIO.setIgnoredAprilTags(ignoredTags);
            },

            (poseObservation) -> VisionConstants.DEFAULT_STANDARD_DEVIATIONS,
            Optional.of(GLOBAL_LOCALIZATION)
        ),

        /**
         * Uses only turret-mounted AprilTag cameras for precise aiming tasks.
         * Suitable for tasks such as shooting or precise alignment.
         */
        TURRET_HUB_AIMING(
            EnumSet.of(
                CameraName.CLOSE_TURRET_CAMERA, 
                CameraName.FAR_TURRET_CAMERA
            ),

            poseObservation -> {
                TrackedAprilTag[] tags = poseObservation.usedAprilTags();
                
                double ambiguity = VisionUtils.getLowestAmbiguity(tags);
                double distance = VisionUtils.getLowestDistanceToCamera(tags);

                double maxAmbiguity = tags.length > 1 ? 0.10 : 0.07;
                double maxDistance = tags.length > 1 ? Units.feetToMeters(21.5) : Units.feetToMeters(15); 

                return ambiguity <= maxAmbiguity && distance <= maxDistance;
            },

            aprilTagIO -> {
                if (aprilTagIO instanceof AprilTagIOPhotonVision || aprilTagIO instanceof AprilTagIOPhotonSim) {
                    aprilTagIO.setPoseObservationType(PoseObservationType.MULTI_TAG_PNP_ON_COPROCESSOR);
                    aprilTagIO.setSecondaryPoseObservationType(PoseObservationType.LOWEST_AMBIGUITY);
                }
                
                Set<Integer> ignoredTags = new HashSet<Integer>();
                ignoredTags.addAll(RED_TRENCH_TAGS);
                ignoredTags.addAll(BLUE_TRENCH_TAGS);
                ignoredTags.addAll(RED_OUTPOST_TAGS);
                ignoredTags.addAll(BLUE_OUTPOST_TAGS);
                ignoredTags.addAll(RED_TOWER_TAGS);
                ignoredTags.addAll(BLUE_TOWER_TAGS);
                
                if (FieldConstants.isRed()) {
                    ignoredTags.addAll(BLUE_HUB_TAGS);
                } else {
                    ignoredTags.addAll(RED_HUB_TAGS);
                }

                aprilTagIO.setIgnoredAprilTags(ignoredTags);
            },

            (poseObservation) -> VisionConstants.DEFAULT_STANDARD_DEVIATIONS,

            Optional.of(STANDARD_HUB_AIM)
        ),

        TOWER_ALIGNMENT(
            EnumSet.of(
                CameraName.INTAKE_LEFT_CAMERA, 
                CameraName.INTAKE_RIGHT_CAMERA, 
                CameraName.CLOSE_TURRET_CAMERA, 
                CameraName.FAR_TURRET_CAMERA, 
                CameraName.BACK_CAMERA
            ),
            poseObservation -> {
                TrackedAprilTag[] tags = poseObservation.usedAprilTags();
                
                double ambiguity = VisionUtils.getLowestAmbiguity(tags);
                double distance = VisionUtils.getLowestDistanceToCamera(tags);

                double maxAmbiguity = tags.length > 1 ? 0.10 : 0.07;
                double maxDistance = tags.length > 1 ? Units.feetToMeters(10) : Units.feetToMeters(7); 

                return ambiguity <= maxAmbiguity && distance <= maxDistance;
            },
             aprilTagIO -> {
                if (aprilTagIO instanceof AprilTagIOPhotonVision || aprilTagIO instanceof AprilTagIOPhotonSim) {
                    aprilTagIO.setPoseObservationType(PoseObservationType.PNP_DISTANCE_TRIG_SOLVE);
                } 
                Set<Integer> ignoredTags = new HashSet<Integer>();
                ignoredTags.addAll(RED_HUB_TAGS);
                ignoredTags.addAll(BLUE_HUB_TAGS);
                ignoredTags.addAll(RED_OUTPOST_TAGS);
                ignoredTags.addAll(BLUE_OUTPOST_TAGS);
                ignoredTags.addAll(RED_TRENCH_TAGS);
                ignoredTags.addAll(BLUE_TRENCH_TAGS);
                
                if (FieldConstants.isRed()) {
                    ignoredTags.addAll(BLUE_TOWER_TAGS);
                } else {
                    ignoredTags.addAll(RED_TOWER_TAGS);
                }

                aprilTagIO.setIgnoredAprilTags(ignoredTags);
            },

            (poseObservation) -> VisionConstants.DEFAULT_STANDARD_DEVIATIONS,

            Optional.of(GLOBAL_LOCALIZATION) 
        ),

        OUTPOST_ALIGNMENT(
            EnumSet.of(
                CameraName.CLOSE_TURRET_CAMERA, 
                CameraName.FAR_TURRET_CAMERA,
                CameraName.INTAKE_LEFT_CAMERA,
                CameraName.INTAKE_RIGHT_CAMERA,
                CameraName.BACK_CAMERA
            ),
            poseObservation -> {
                TrackedAprilTag[] tags = poseObservation.usedAprilTags();
                
                double ambiguity = VisionUtils.getLowestAmbiguity(tags);
                double distance = VisionUtils.getLowestDistanceToCamera(tags);

                double maxAmbiguity = tags.length > 1 ? 0.10 : 0.07;
                double maxDistance = tags.length > 1 ? Units.feetToMeters(10) : Units.feetToMeters(7); 

                return ambiguity <= maxAmbiguity && distance <= maxDistance;
            },
             aprilTagIO -> {
                if (aprilTagIO instanceof AprilTagIOPhotonVision || aprilTagIO instanceof AprilTagIOPhotonSim) {
                    aprilTagIO.setPoseObservationType(PoseObservationType.MULTI_TAG_PNP_ON_COPROCESSOR);
                    aprilTagIO.setSecondaryPoseObservationType(PoseObservationType.LOWEST_AMBIGUITY);
                } 
                Set<Integer> ignoredTags = new HashSet<Integer>();
                ignoredTags.addAll(RED_HUB_TAGS);
                ignoredTags.addAll(BLUE_HUB_TAGS);
                ignoredTags.addAll(RED_OUTPOST_TAGS);
                ignoredTags.addAll(BLUE_OUTPOST_TAGS);
                ignoredTags.addAll(RED_TRENCH_TAGS);
                ignoredTags.addAll(BLUE_TRENCH_TAGS);
                
                if (FieldConstants.isRed()) {
                    ignoredTags.addAll(BLUE_HUB_TAGS);
                } else {
                    ignoredTags.addAll(RED_HUB_TAGS);
                }

                aprilTagIO.setIgnoredAprilTags(ignoredTags);
            },

            (poseObservation) -> VisionConstants.DEFAULT_STANDARD_DEVIATIONS,
            Optional.of(GLOBAL_LOCALIZATION) 
        );

        
        @Getter private final EnumSet<CameraName> camerasToUse;
        @Getter private final Predicate<PoseObservation> cameraFilter;
        @Getter private final Consumer<AprilTagIO> cameraConfiguration;
        @Getter private final Function<PoseObservation, Matrix<N3, N1>> standardDeviationCalculator;
        @Getter private final Optional<AprilTagGoal> backupGoal;
        
        /**
         * @param camerasToUse Supplier that provides the set of cameras to use for this goal
         * @param cameraFilter Predicate that takes the pose observation of a camera to determine if it should be used for localization
         * @param cameraConfiguration Consumer that configures the AprilTagIO for this goal
         * @param standardDeviationCalculator Function that provides the standard deviations for the pose observation
         */
        private AprilTagGoal(
            EnumSet<CameraName> camerasToUse, 
            Predicate<PoseObservation> cameraFilter, 
            Consumer<AprilTagIO> cameraConfiguration, 
            Function<PoseObservation, Matrix<N3, N1>> standardDeviationCalculator,
            Optional<AprilTagGoal> backupGoal
        ) {
            this.camerasToUse = camerasToUse;
            this.cameraFilter = cameraFilter;
            this.cameraConfiguration = cameraConfiguration;
            this.standardDeviationCalculator = standardDeviationCalculator;
            this.backupGoal = backupGoal;
        }
    }
}