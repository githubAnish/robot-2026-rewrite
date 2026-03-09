package org.frogforce503.robot.auto;

import java.util.ArrayList;
import java.util.List;

import org.frogforce503.lib.auto.pathplanner.LocalADStarAK;
import org.frogforce503.lib.auto.pathplanner.PathPlannerUtil;
import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.auto.autos.RandomAuto;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoChooser {
    private final Drive drive;
    private final Vision vision;

    private final LoggedDashboardChooser<AutoMode> routineChooser = new LoggedDashboardChooser<>("Auto");

    private Command autoCommand;
    private AutoMode lastSelectedAuto;

    public AutoChooser(Drive drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;

        // Configure PathPlanner
        PathPlannerUtil.configureAutoBuilder(drive);
        Pathfinding.setPathfinder(new LocalADStarAK());

        // Configure autos
        configureAutos();
    }

    private void configureAutos() {
        // Random test auto
        routineChooser.addDefaultOption(
            "Test",
            new AutoMode() {
                @Override
                public Command getCommand() {
                    return Commands.runOnce(() -> drive.setPose(GeomUtil.toPose2d(new Translation2d(1.889,4.002))));
                }

                @Override
                public List<Pose2d> getPoses() {
                    return new ArrayList<>(List.of(Pose2d.kZero));
                }
        });

        routineChooser.addOption("Random", new RandomAuto());
    }

    public void startAuto() {
        final AutoMode selectedAuto = routineChooser.get();

        if (selectedAuto == null) {
            return;
        }

        final String autoName = selectedAuto.getClass().getSimpleName();

        autoCommand = selectedAuto.getCommand().withName(autoName);

        if (autoCommand != null) {
            CommandScheduler.getInstance().schedule(autoCommand);
        }
    }

    public void periodic() {
        final AutoMode selectedAuto = routineChooser.get();

        if (selectedAuto == null) {
            logTrajectory(); // Clear poses

        } else if (selectedAuto != lastSelectedAuto) {
            List<Pose2d> trajectoryPoses = selectedAuto.getPoses();
            Pose2d start = trajectoryPoses.get(0);

            logTrajectory(trajectoryPoses.toArray(Pose2d[]::new));
        
            drive.setPose(start);
        }

        lastSelectedAuto = selectedAuto;
    }

    public void close() {
        logTrajectory(); // Clear poses

        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    private void logTrajectory(Pose2d... trajectory) {
        drive.getViz().getObject("Trajectory").setPoses(trajectory);
        Logger.recordOutput("Drive/Trajectory", trajectory);
    }
}