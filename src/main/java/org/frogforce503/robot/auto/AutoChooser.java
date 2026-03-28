package org.frogforce503.robot.auto;

import org.frogforce503.lib.auto.bline.BLineUtil;
import org.frogforce503.lib.auto.pathplanner.LocalADStarAK;
import org.frogforce503.lib.auto.pathplanner.PathPlannerUtil;
import org.frogforce503.robot.auto.test.PutRobotInsideMapleSimField;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.BLine.FollowPath;
import lombok.Getter;

public class AutoChooser {
    private final Drive drive;
    @Getter private final FollowPath.Builder blineAutoBuilder;

    private final LoggedDashboardChooser<AutoMode> routineChooser = new LoggedDashboardChooser<>("Auto");

    private Command autoCommand;
    private AutoMode lastSelectedAuto;

    private final double simAutoTimeSec = 20;

    public AutoChooser(Drive drive) {
        this.drive = drive;

        // Configure PathPlanner
        PathPlannerUtil.configureAutoBuilder(drive);
        Pathfinding.setPathfinder(new LocalADStarAK());

        // Configure BLine
        blineAutoBuilder = BLineUtil.configureAutoBuilder(drive);

        // Configure autos
        if (RobotBase.isSimulation()) {
            routineChooser.addDefaultOption("Put Robot Inside MapleSim Field", new PutRobotInsideMapleSimField(drive));
        }
    }

    public void addAuto(String name, AutoMode autoMode) {
        routineChooser.addOption(name, autoMode);
    }

    public void startAuto() {
        final AutoMode selectedAuto = routineChooser.get();

        if (selectedAuto == null) {
            return;
        }

        autoCommand =
            selectedAuto
                .getCommand()
                .withName(selectedAuto.getClass().getSimpleName());

        if (RobotBase.isSimulation()) {
            autoCommand = autoCommand.withTimeout(simAutoTimeSec);
        }

        if (autoCommand != null) {
            CommandScheduler.getInstance().schedule(autoCommand);
        }
    }

    public void periodic() {
        final AutoMode selectedAuto = routineChooser.get();

        if (selectedAuto == null) {
            logTrajectory(); // Clear poses

        } else if (selectedAuto != lastSelectedAuto) {
            Pose2d[] trajectoryPoses = selectedAuto.getPoses();

            drive.setPose(trajectoryPoses[0]); // Set drive pose to trajectory start
            logTrajectory(trajectoryPoses);
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