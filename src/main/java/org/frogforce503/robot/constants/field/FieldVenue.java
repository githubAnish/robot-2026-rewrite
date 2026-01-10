package org.frogforce503.robot.constants.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import lombok.Getter;

public enum FieldVenue {
    Shop("Shop.json", AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));

    @Getter private final String filePath;
    @Getter private final AprilTagFieldLayout aprilTagFieldLayout;

    private FieldVenue(String filePath, AprilTagFieldLayout aprilTagFieldLayout) {
        this.filePath = filePath;
        this.aprilTagFieldLayout = aprilTagFieldLayout;
    }
}
