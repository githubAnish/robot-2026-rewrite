package org.frogforce503.robot.constants.field;

import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.FieldConstantsUtil;
import org.frogforce503.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

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
        // blue hub center translation2d, x coord id 18 y coord id 26
        public static final Translation2d blueCenter;

        static {
            blueCenter = new Translation2d(getTagPose2d(18).getX(), getTagPose2d(26).getY());
        }
    }

    public static class Outpost {
        public static final Pose2d blue = getTagPose2d(29);
    }
}
