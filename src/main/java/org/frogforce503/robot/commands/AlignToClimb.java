package org.frogforce503.robot.commands;

import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AlignToClimb extends SequentialCommandGroup {
    public AlignToClimb(Drive drive) {
        addCommands(
            new DriveToPose(drive, () -> FieldConstants.Tower.getPreClimbPose(drive.getPose())),
            new DriveToPose(drive, () -> FieldConstants.Tower.getClimbPose(drive.getPose())));
    }
}