package org.frogforce503.robot.auto;

import java.util.List;

import org.frogforce503.lib.auto.pathplanner.LocalADStarAK;
import org.frogforce503.lib.auto.pathplanner.PathPlannerUtil;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutoChooser {
    private final Drive drive;

    private final LoggedDashboardChooser<AutoMode> routineChooser = new LoggedDashboardChooser<>("Auto");

    private Command autoCommand;
    private AutoMode lastSelectedAuto;

    public AutoChooser(
        Drive drive
    ) {
        this.drive = drive;

        // Configure PathPlanner
        PathPlannerUtil.configureAutoBuilder(drive);
        Pathfinding.setPathfinder(new LocalADStarAK());
    }

    public void addAuto(String autoName, AutoMode auto) {
        routineChooser.addOption(autoName, auto);
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