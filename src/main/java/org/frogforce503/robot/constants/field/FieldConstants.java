package org.frogforce503.robot.constants.field;

import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.FieldConstantsUtil;
import org.frogforce503.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

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
        public static final double blueInitLineX = 0.0;
        public static final double redInitLineX = 0.0;

        static {
            // final double BlueInitLineToLeftCage = FieldConstantsUtil.getFieldValueMeters("BlueInitLineToLeftCage");
            // blueInitLineX = fieldLength / 2 - BlueInitLineToLeftCage;

            // final double RedInitLineToLeftCage = FieldConstantsUtil.getFieldValueMeters("RedInitLineToLeftCage");
            // redInitLineX = fieldLength / 2 + RedInitLineToLeftCage;
        }
    }

    public static class Hub {

    }

    public static class Outpost {
        // center line to trench center
        // trench center to init line
        
        static {
            
        }
    }
}
