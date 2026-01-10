package org.frogforce503.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Class handling alliance context and {@link Field2d} visualization. */
public final class FieldInfo {
    private static final Field2d field2d = new Field2d();

    private FieldInfo() {}

    static {
        // Set default alliance
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

        SmartDashboard.putData("Field", field2d);
    }

    /** Returns current alliance. */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    /** Returns if currently on red alliance. */
    public static boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    /** Sets the robot pose on Field2d. */
    public static void setRobotPose(Pose2d robotPose) {
        field2d.setRobotPose(robotPose);
    }

    /** Gets or creates a field object on Field2d. */
    public static FieldObject2d getObject(String name) {
        return field2d.getObject(name);
    }
}