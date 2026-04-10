package org.frogforce503.robot.constants.field;

import java.util.Set;

import org.frogforce503.lib.math.AllianceFlipUtil;
import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.Zone;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double fieldLength = aprilTagFieldLayout.getFieldLength();
    public static final double fieldWidth = aprilTagFieldLayout.getFieldWidth();

    /** Returns current alliance. */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    /** Returns if currently on red alliance. */
    public static boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    public static Pose2d getTagPose2d(int tagId) {
        return
            aprilTagFieldLayout
                .getTagPose(tagId)
                .orElseThrow(() -> new IllegalArgumentException("No tag with ID " + tagId + " found in layout" + ErrorUtil.attachJavaClassName(FieldConstants.class)))
                .toPose2d();
    }

    public static boolean inAllianceZone(Pose2d robotPose) {
        return AllianceFlipUtil.applyX(robotPose.getX()) < Lines.blueInitLineX;
    }

    /**
     * Returns the hub shot pose when the robot is in the alliance zone;
     * otherwise returns the nearest lob shot pose (Depot or Outpost).
     */
    public static Translation3d getShotTarget(Pose2d robotPose) {
        return
            inAllianceZone(robotPose)
                ? Hub.getHubShotPose()
                : new Translation3d(robotPose.getTranslation().nearest(Set.of(Depot.getLobShotPose(), Outpost.getLobShotPose())));
    }

    public static class Lines {
        public static final double blueInitLineX = Units.inchesToMeters(156.8); // Alliance Wall To Init Line
    }

    public static class Hub {
        public static final Translation3d blueCenter;
        public static final Translation3d blueShotPose;

        static {
            final double hubHeight = Units.inchesToMeters(72.0);
            final double hubHeightToShotHeight = Units.inchesToMeters(10.0);

            blueCenter = new Translation3d(getTagPose2d(18).getX(), getTagPose2d(26).getY(), hubHeight);
            blueShotPose = blueCenter.plus(new Translation3d(0.0, 0.0, -hubHeightToShotHeight));
        }

        private static Translation3d getHubShotPose() {
            return AllianceFlipUtil.apply(blueShotPose);
        }
    }

    public static class Outpost {
        public static final Pose2d blue = getTagPose2d(29);

        private static Translation2d getLobShotPose() {
            return
                AllianceFlipUtil.apply(
                    blue
                        .plus(GeomUtil.toTransform2d(Units.inchesToMeters(36), Units.inchesToMeters(18))) // Lob Shot Pose Offset
                        .getTranslation());
        }
    }

    public static class Depot {
        public static final Zone blue;

        static {
            final double depotLength = Units.inchesToMeters(26.7);
            final double depotWidth = Units.inchesToMeters(42.0);
            final double wallToDepotY = Units.inchesToMeters(61.5);

            // Blue Depot
            Translation2d blueBackLeftCorner = new Translation2d(0, fieldWidth - wallToDepotY);
            Translation2d blueFrontRightCorner = blueBackLeftCorner.plus(new Translation2d(depotLength, -depotWidth));

            blue = new Zone(blueBackLeftCorner, blueFrontRightCorner);
        }

        private static Translation2d getLobShotPose() {
            return
                AllianceFlipUtil.apply(
                    blue
                        .getCenter()
                        .plus(GeomUtil.toTransform2d(Units.inchesToMeters(36), 0)) // Lob Shot Pose Offset
                        .getTranslation());
        }
    }

    /**
     * <p> Defines the bounding box that all fuel (only in neutral zone) is corralled into before start of match. </p> 
     * <b> All corners must be viewed from the blue alliance. </b>
     */
    public static class NeutralZone {
        public static final Zone zone;

        static {
            final double boundingBoxWidth = Units.inchesToMeters(206.0); // See frcmanual.com
            final double boundingBoxDepth = Units.inchesToMeters(72.0); // See frcmanual.com
            final Translation2d fieldCenter = new Translation2d(fieldLength / 2, fieldWidth / 2);
            
            Translation2d frontLeftCorner = fieldCenter.plus(new Translation2d(boundingBoxDepth / 2, boundingBoxWidth / 2));
            Translation2d backRightCorner = fieldCenter.plus(new Translation2d(-boundingBoxDepth / 2, -boundingBoxWidth / 2));

            zone = new Zone(backRightCorner, frontLeftCorner);
        }
    }

    public static class Tower {
        public static final Zone blue;

        private static final Pose2d blueLeftPreClimbPose;
        private static final Pose2d blueRightPreClimbPose;

        private static final Pose2d blueLeftClimbPose;
        private static final Pose2d blueRightClimbPose;

        static {
            final double rungLength = Units.inchesToMeters(39); // See frcmanual.com
            final double centerTagToTowerX = Units.inchesToMeters(41.86); // See field CAD

            // Tower
            Pose2d blueTowerTag = getTagPose2d(31);
            Translation2d blueBackLeftCorner = blueTowerTag.getTranslation().plus(new Translation2d(0, rungLength / 2));
            Translation2d blueFrontRightCorner = blueTowerTag.getTranslation().plus(new Translation2d(centerTagToTowerX, -rungLength / 2));

            blue = new Zone(blueBackLeftCorner, blueFrontRightCorner);

            // Climb Poses
            final double towerCornerToRobotClimbPoseOffset = Units.inchesToMeters(20);
            final double preClimbtoClimbPoseOffset = Units.inchesToMeters(5);

            blueLeftClimbPose =
                new Pose2d(blue.getFrontLeftCorner(), Rotation2d.kZero)
                    .plus(GeomUtil.toTransform2d(0, towerCornerToRobotClimbPoseOffset));
                    
            blueRightClimbPose =
                new Pose2d(blue.getFrontRightCorner(), Rotation2d.kZero)
                    .plus(GeomUtil.toTransform2d(0, -towerCornerToRobotClimbPoseOffset))
                    .plus(GeomUtil.toTransform2d(Rotation2d.k180deg));

            blueLeftPreClimbPose =
                blueLeftClimbPose
                    .plus(GeomUtil.toTransform2d(0, preClimbtoClimbPoseOffset));

            blueRightPreClimbPose =
                blueRightClimbPose
                    .plus(GeomUtil.toTransform2d(0, -preClimbtoClimbPoseOffset));
        }

        public static Pose2d getPreClimbPose(Pose2d robotPose) {
            return
                AllianceFlipUtil.apply(
                    robotPose.nearest(Set.of(blueLeftPreClimbPose, blueRightPreClimbPose)));
        }

        public static Pose2d getClimbPose(Pose2d robotPose) {
            return
                AllianceFlipUtil.apply(
                    robotPose.nearest(Set.of(blueLeftClimbPose, blueRightClimbPose)));
        }
    }

    public static class Trench {
        public static final Zone blueLeft;
        public static final Zone blueRight;

        static {
            final double trenchLength = Units.inchesToMeters(49.0);
            final double trenchWidth = Units.inchesToMeters(63.0);

            // Blue Left Trench
            Translation2d blueLeftBackLeftCorner = new Translation2d(Lines.blueInitLineX, fieldWidth);
            Translation2d blueLeftFrontRightCorner = blueLeftBackLeftCorner.plus(new Translation2d(trenchLength, -trenchWidth));

            blueLeft = new Zone(blueLeftBackLeftCorner, blueLeftFrontRightCorner);

            // Blue Right Trench
            Translation2d blueRightBackRightCorner = new Translation2d(Lines.blueInitLineX, 0.0);
            Translation2d blueRightFrontLeftCorner = blueRightBackRightCorner.plus(new Translation2d(trenchLength, trenchWidth));

            blueRight = new Zone(blueRightBackRightCorner, blueRightFrontLeftCorner);
        }
    }

    public static class Bump {
        public static final Zone blueLeft;
        public static final Zone blueRight;

        static {
            final double bumpLength = Units.inchesToMeters(49.0);
            final double bumpWidth = Units.inchesToMeters(73.0);
            final double trenchWidth = Units.inchesToMeters(63.0);

            // Blue Left Bump
            Translation2d blueLeftBackLeftCorner = new Translation2d(Lines.blueInitLineX, fieldWidth - trenchWidth);
            Translation2d blueLeftFrontRightCorner = blueLeftBackLeftCorner.plus(new Translation2d(bumpLength, -bumpWidth));

            blueLeft = new Zone(blueLeftBackLeftCorner, blueLeftFrontRightCorner);

            // Blue Right Bump
            Translation2d blueRightBackRightCorner = new Translation2d(Lines.blueInitLineX, trenchWidth);
            Translation2d blueRightFrontLeftCorner = blueRightBackRightCorner.plus(new Translation2d(bumpLength, bumpWidth));

            blueRight = new Zone(blueRightBackRightCorner, blueRightFrontLeftCorner);
        }
    }
}