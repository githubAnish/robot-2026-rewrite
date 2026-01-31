package org.frogforce503.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface AutoMode {
    public abstract Command getCommand();
    public abstract List<Pose2d> getPoses();
}