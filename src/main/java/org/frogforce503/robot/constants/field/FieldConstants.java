package org.frogforce503.robot.constants.field;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
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
    public static final AprilTagFieldLayout aprilTagFieldLayout = Constants.fieldVenue.getAprilTagFieldLayout();

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

    public static class Lines {
        public static final double blueInitLineX;
        public static final double redInitLineX;

        static {
            final double allianceWallToBlueInitLine = Units.inchesToMeters(156.8);
            blueInitLineX = allianceWallToBlueInitLine;

            final double allianceWallToRedInitLine = Units.inchesToMeters(156.8);
            redInitLineX = fieldLength - allianceWallToRedInitLine;
        }
    }

    public static class Hub {
        public static final Translation3d blueCenter;
        public static final Translation3d redCenter;

        public static final Translation3d blueShotPose;
        public static final Translation3d redShotPose;

        static {
            final double hubHeight = Units.inchesToMeters(72.0);
            final double hubHeightToShotHeight = Units.inchesToMeters(10.0);

            blueCenter = new Translation3d(getTagPose2d(18).getX(), getTagPose2d(26).getY(), hubHeight);
            redCenter = new Translation3d(getTagPose2d(2).getX(), getTagPose2d(10).getY(), hubHeight);

            blueShotPose = blueCenter.plus(new Translation3d(0.0, 0.0, -hubHeightToShotHeight));
            redShotPose = redCenter.plus(new Translation3d(0.0, 0.0, -hubHeightToShotHeight));
        }
    }

    public static class Outpost {
        public static final Pose2d blue = getTagPose2d(29);
        public static final Pose2d red = getTagPose2d(13);
    }

    public static class Depot {
        public static final Translation2d blueFrontLeftCorner;
        public static final Translation2d blueFrontRightCorner;
        public static final Translation2d blueBackLeftCorner;
        public static final Translation2d blueBackRightCorner;

        public static final Translation2d redFrontLeftCorner;
        public static final Translation2d redFrontRightCorner;
        public static final Translation2d redBackLeftCorner;
        public static final Translation2d redBackRightCorner;

        static {
            final double backLeftToFrontLeft = Units.inchesToMeters(26.7);
            final double frontLeftToFrontRight = Units.inchesToMeters(42.0);

            final double leftWallToBlueBackLeft = Units.inchesToMeters(61.5);

            blueBackLeftCorner = new Translation2d(0, fieldWidth - leftWallToBlueBackLeft);
            blueFrontLeftCorner = blueBackLeftCorner.plus(new Translation2d(backLeftToFrontLeft, 0));
            blueFrontRightCorner = blueFrontLeftCorner.plus(new Translation2d(0, -frontLeftToFrontRight));
            blueBackRightCorner = blueFrontRightCorner.plus(new Translation2d(-backLeftToFrontLeft, 0));

            final double rightWallToRedBackLeft = Units.inchesToMeters(61.5);
            
            redBackLeftCorner = new Translation2d(fieldLength, rightWallToRedBackLeft);
            redFrontLeftCorner = redBackLeftCorner.plus(new Translation2d(-backLeftToFrontLeft, 0));
            redFrontRightCorner = redFrontLeftCorner.plus(new Translation2d(0, frontLeftToFrontRight));
            redBackRightCorner = redFrontRightCorner.plus(new Translation2d(backLeftToFrontLeft, 0));
        }
    }

    /**
     * <p> Defines the bounding box that all fuel (only in neutral zone) is corralled into before start of match. </p> 
     * <b> All corners must be viewed from the blue alliance. </b>
     */
    public static class NeutralZone {
        public static final Translation2d frontLeftCorner;
        public static final Translation2d frontRightCorner;
        public static final Translation2d backLeftCorner;
        public static final Translation2d backRightCorner;

        static {
            final double boundingBoxWidth = Units.inchesToMeters(206.0); // From game manual, see https://www.frcmanual.com/2026/game-details#_6341-neutral-zone-fuel-arrangement
            final double boundingBoxDepth = Units.inchesToMeters(72.0); // From game manual, see https://www.frcmanual.com/2026/game-details#_6341-neutral-zone-fuel-arrangement

            final Translation2d fieldCenter = new Translation2d(fieldLength / 2, fieldWidth / 2);
            
            frontLeftCorner = fieldCenter.plus(new Translation2d(boundingBoxDepth / 2, boundingBoxWidth / 2));
            frontRightCorner = fieldCenter.plus(new Translation2d(boundingBoxDepth / 2, -boundingBoxWidth / 2));
            backLeftCorner = fieldCenter.plus(new Translation2d(-boundingBoxDepth / 2, boundingBoxWidth / 2));
            backRightCorner = fieldCenter.plus(new Translation2d(-boundingBoxDepth / 2, -boundingBoxWidth / 2));
        }
    }

    public static class Tower {
        public static final Pose2d blueCenter;
        public static final Pose2d blueLeft;
        public static final Pose2d blueRight;

        public static final Pose2d redCenter;
        public static final Pose2d redLeft;
        public static final Pose2d redRight;

        static {
            final double rungLength = Units.inchesToMeters(41.1); // from measuring on field CAD onshape
            final double centerTagToTowerX = Units.inchesToMeters(41.86); // from measuring on field CAD onshape

            blueCenter = getTagPose2d(31).plus(GeomUtil.toTransform2d(centerTagToTowerX, 0));
            blueLeft = blueCenter.plus(GeomUtil.toTransform2d(0, rungLength / 2));
            blueRight = blueCenter.plus(GeomUtil.toTransform2d(0, -rungLength / 2));

            redCenter = getTagPose2d(15).plus(GeomUtil.toTransform2d(centerTagToTowerX, 0));
            redLeft = redCenter.plus(GeomUtil.toTransform2d(0, rungLength / 2));
            redRight = redCenter.plus(GeomUtil.toTransform2d(0, -rungLength / 2));
        }

        /** Interpolated climb pose between blueLeft (t = 0) and blueRight (t = 1). */
        public static Pose2d getBlueClimbPose(double t) {
            return blueLeft.interpolate(blueRight, t);
        }

        /** Interpolated climb pose between redLeft (t = 0) and redRight (t = 1). */
        public static Pose2d getRedClimbPose(double t) {
            return redLeft.interpolate(redRight, t);
        }
    }

    public static class LeftTrench {
        public static final Translation2d blueLeftTrenchCenter = getTagPose2d(23).getTranslation();
        public static final Translation2d redLeftTrenchCenter = getTagPose2d(7).getTranslation();
    }

    public static class RightTrench {
        public static final Translation2d blueRightTrenchCenter = getTagPose2d(28).getTranslation();
        public static final Translation2d redRightTrenchCenter = getTagPose2d(12).getTranslation();
    }

    public static class LeftBump {
        public static final Translation2d blueFrontLeftCorner;
        public static final Translation2d blueFrontRightCorner;
        public static final Translation2d blueBackLeftCorner;
        public static final Translation2d blueBackRightCorner;

        public static final Translation2d redFrontLeftCorner;
        public static final Translation2d redFrontRightCorner;
        public static final Translation2d redBackLeftCorner;
        public static final Translation2d redBackRightCorner;

        static {
            final double backLeftToFrontLeft = Units.inchesToMeters(49.0);
            final double frontLeftToFrontRight = Units.inchesToMeters(73.0);

            // Blue Left Bump
            final double leftWallToBlueLeftBumpBackLeft = Units.inchesToMeters(63.0);

            blueBackLeftCorner = new Translation2d(Lines.blueInitLineX, fieldWidth - leftWallToBlueLeftBumpBackLeft);
            blueFrontLeftCorner = blueBackLeftCorner.plus(new Translation2d(backLeftToFrontLeft, 0));
            blueFrontRightCorner = blueFrontLeftCorner.plus(new Translation2d(0, -frontLeftToFrontRight));
            blueBackRightCorner = blueFrontRightCorner.plus(new Translation2d(-backLeftToFrontLeft, 0));

            // Red Left Bump
            final double rightWallToRedLeftBumpBackLeft = Units.inchesToMeters(63.0);

            redBackLeftCorner = new Translation2d(Lines.redInitLineX, rightWallToRedLeftBumpBackLeft);
            redFrontLeftCorner = redBackLeftCorner.plus(new Translation2d(-backLeftToFrontLeft, 0));
            redFrontRightCorner = redFrontLeftCorner.plus(new Translation2d(0, frontLeftToFrontRight));
            redBackRightCorner = redFrontRightCorner.plus(new Translation2d(backLeftToFrontLeft, 0));
        }
    }

    public static class RightBump {
        public static final Translation2d blueFrontLeftCorner;
        public static final Translation2d blueFrontRightCorner;
        public static final Translation2d blueBackLeftCorner;
        public static final Translation2d blueBackRightCorner;

        public static final Translation2d redFrontLeftCorner;
        public static final Translation2d redFrontRightCorner;
        public static final Translation2d redBackLeftCorner;
        public static final Translation2d redBackRightCorner;

        static {
            final double backLeftToFrontLeft = Units.inchesToMeters(49.0);
            final double frontLeftToFrontRight = Units.inchesToMeters(73.0);

            // Blue Right Bump
            final double rightWallToBlueRightBumpBackRight = Units.inchesToMeters(63.0);

            blueBackRightCorner = new Translation2d(Lines.blueInitLineX, rightWallToBlueRightBumpBackRight);
            blueFrontRightCorner = blueBackRightCorner.plus(new Translation2d(backLeftToFrontLeft, 0));
            blueFrontLeftCorner = blueFrontRightCorner.plus(new Translation2d(0, frontLeftToFrontRight));
            blueBackLeftCorner = blueFrontLeftCorner.plus(new Translation2d(-backLeftToFrontLeft, 0));

            // Red Right Bump
            final double leftWallToRedRightBumpBackRight = Units.inchesToMeters(63.0);

            redBackRightCorner = new Translation2d(Lines.redInitLineX, fieldWidth - leftWallToRedRightBumpBackRight);
            redFrontRightCorner = redBackRightCorner.plus(new Translation2d(-backLeftToFrontLeft, 0));
            redFrontLeftCorner = redFrontRightCorner.plus(new Translation2d(0, -frontLeftToFrontRight));
            redBackLeftCorner = redFrontLeftCorner.plus(new Translation2d(backLeftToFrontLeft, 0));
        }
    }
}
