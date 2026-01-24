package org.frogforce503.robot.auto.autos.test;

import java.io.IOException;
import java.util.List;

import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.robot.auto.AutoMode;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveStraightTest implements AutoMode {
    private PathPlannerPath driveStraightPath;

    public DriveStraightTest(Drive drive, Vision vision, Superstructure superstructure) {
        try {
            driveStraightPath = PathPlannerPath.fromPathFile("Test");
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.println("Error creating auto" + ErrorUtil.attachJavaClassName(DriveStraightTest.class));
            e.printStackTrace();
        }
    }

    @Override
    public Command getCommand() {
        return AutoBuilder.followPath(driveStraightPath);
    }

    @Override
    public List<Pose2d> getPoses() {
        return driveStraightPath.getPathPoses();
    }
}
