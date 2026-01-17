package org.frogforce503.robot.constants.field;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.FieldConstantsUtil;
import org.frogforce503.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;

public class FieldConstants {
    public static final AprilTagFieldLayout aprilTagFieldLayout = Constants.fieldVenue.getAprilTagFieldLayout();

    public static final double fieldLength = aprilTagFieldLayout.getFieldLength();
    public static final double fieldWidth = aprilTagFieldLayout.getFieldWidth();

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
            final double allianceWallToBlueInitLine = FieldConstantsUtil.getFieldValueMeters("AllianceWallToBlueInitLine");
            blueInitLineX = allianceWallToBlueInitLine;

            final double allianceWallToRedInitLine = FieldConstantsUtil.getFieldValueMeters("AllianceWallToRedInitLine");
            redInitLineX = allianceWallToRedInitLine;
        }
    }

    public static class Hub {
        public static final Translation3d blueCenter;
        public static final Translation3d redCenter;

        public static final Translation3d blueShotPose;
        public static final Translation3d redShotPose;

        static {
            final double hubHeight = Units.inchesToMeters(72.0); // account for shot pose to be 10 inches below
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
        // public static final Pose2d blue = getTagPose2d(29);
        // public static final Pose2d red = getTagPose2d(13);
        // basically get a reference pose (maybe a certain dist from the right wall) so you can apply a few offsets for auto
        // get corners
        public static final Translation2d blueFrontLeftCorner;
        public static final Translation2d blueFrontRightCorner;
        public static final Translation2d blueBackLeftCorner;
        public static final Translation2d blueBackRightCorner;

        static {
            final double leftWallToBlueBackLeft = FieldConstantsUtil.getFieldValueMeters("LeftWallToBlueBackLeft");

            blueBackLeftCorner = new Translation2d(0, fieldWidth - leftWallToBlueBackLeft);
            blueFrontLeftCorner = blueBackLeftCorner.plus(new Translation2d(0, 0));
            blueFrontRightCorner = blueFrontLeftCorner.plus(new Translation2d(0, 0));
            blueBackRightCorner = blueFrontLeftCorner.plus(new Translation2d(0, 0));
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
            redLeft = redCenter.plus(GeomUtil.toTransform2d(0, -rungLength / 2));
            redRight = redCenter.plus(GeomUtil.toTransform2d(0, rungLength / 2));
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

    public static class Trench {
        // get the center waypoint of the trench
        public static final Translation2d blueLeftTrenchCenter = getTagPose2d(23).getTranslation();
        public static final Translation2d blueRightTrenchCenter = getTagPose2d(28).getTranslation();

        public static final Translation2d redLeftTrenchCenter = getTagPose2d(7).getTranslation();
        public static final Translation2d redRightTrenchCenter = getTagPose2d(12).getTranslation();
    }

    public static class Bump {
        // get the vertices and define the region of the bumps

    }

    public static class NeutralZone {
        // define the regions of balls in neutral zone
    }
}
