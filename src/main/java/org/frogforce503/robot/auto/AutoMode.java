package org.frogforce503.robot.auto;

import java.util.List;

import org.frogforce503.lib.auto.planned_path.PlannedPath;
import org.frogforce503.robot.commands.drive.DrivePlannedPath;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.vision.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.Getter;

public abstract class AutoMode {
    private final Drive drive;
    private final Vision vision;
    private final Superstructure superstructure;

    @Getter private final String name;

    public AutoMode(Drive drive, Vision vision, Superstructure superstructure) {
        this.drive = drive;
        this.vision = vision;
        this.superstructure = superstructure;

        this.name = this.getClass().getSimpleName();
    }

    public abstract Command getCommand();
    public abstract List<Pose2d> getPoses();

    // Helpful actions
    public Command drive(PlannedPath path) {
        return new DrivePlannedPath(drive, path);
    }
}