package org.frogforce503.robot.auto.autos.blue;

import java.util.ArrayList;
import java.util.List;

import org.frogforce503.lib.auto.pathplanner.PathPlannerUtil;
import org.frogforce503.robot.auto.AutoMode;
import org.frogforce503.robot.commands.IntakeFuelFromGround;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.vision.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// Go to NZ, pick up balls, use pathfindThenFollowPath to go to trench start, then do same thing again, then go to NZ for more
// so you just have 
public class BlueLeftTrenchGoToNZTwice implements AutoMode {
    private final Drive drive;
    private final Vision vision;
    private final Superstructure superstructure;

    private final PathPlannerPath trenchToNZ1;
    private final PathPlannerPath NZToTrench1;
    private final PathPlannerPath trenchToNZ2;
    private final PathPlannerPath NZToTrench2;
    private final PathPlannerPath trenchToNZ3;

    public BlueLeftTrenchGoToNZTwice(Drive drive, Vision vision, Superstructure superstructure) {
        this.drive = drive;
        this.vision = vision;
        this.superstructure = superstructure;

        trenchToNZ1 = PathPlannerUtil.loadChoreoTrajectory("null");
        NZToTrench1 = PathPlannerUtil.loadChoreoTrajectory("null");
        trenchToNZ2 = PathPlannerUtil.loadChoreoTrajectory("null");
        NZToTrench2 = PathPlannerUtil.loadChoreoTrajectory("null");
        trenchToNZ3 = PathPlannerUtil.loadChoreoTrajectory("null");
    }

    @Override
    public Command getCommand() {
        return
            Commands.sequence(
                // AutoBuilder.followPath(trenchToNZ1).alongWith(new IntakeFuelFromGround(drive, vision, superstructure, null, () -> true))
            );
    }

    @Override
    public List<Pose2d> getPoses() {
        return new ArrayList<>();
    }
}
