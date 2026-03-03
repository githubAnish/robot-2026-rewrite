package org.frogforce503.robot.auto.autos;

import java.util.List;

import org.frogforce503.lib.auto.pathplanner.PathPlannerUtil;
import org.frogforce503.robot.auto.AutoMode;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RandomAuto implements AutoMode {
    private PathPlannerPath randomPath;

    public RandomAuto() {
        randomPath = PathPlannerUtil.loadTrajectory("Test");
    }

    @Override
    public Command getCommand() {
        return AutoBuilder.followPath(randomPath);
    }

    @Override
    public List<Pose2d> getPoses() {
        return randomPath.getPathPoses();
    }
}
