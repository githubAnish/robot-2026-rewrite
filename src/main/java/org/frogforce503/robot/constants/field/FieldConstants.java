package org.frogforce503.robot.constants.field;

import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.FieldConstantsUtil;
import org.frogforce503.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

        static {
            final double CenterLineToBlueInitLine = FieldConstantsUtil.getFieldValueMeters("CenterLineToBlueInitLine");
            blueInitLineX = fieldLength / 2 - CenterLineToBlueInitLine;
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
    }

    public static class Tower {
        // get the tag ids and add some offset to get a close enough position to align to
    }

    public static class Trench {
        // get the center waypoint of the trench
    }

    public static class Bump {
        // get the vertices and define the region of the bumps
    }

    public static class NeutralZone {
        // define the regions of balls in neutral zone
    }
}
