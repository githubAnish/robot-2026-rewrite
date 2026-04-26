package org.frogforce503.robot.auto;

import org.frogforce503.lib.auto.bline.BLineUtil;
import org.frogforce503.lib.auto.pathplanner.LocalADStarAK;
import org.frogforce503.lib.auto.pathplanner.PathPlannerUtil;
import org.frogforce503.robot.auto.autos.Left2NZ1678;
import org.frogforce503.robot.auto.autos.PreloadDepotClimb;
import org.frogforce503.robot.auto.autos.Right2NZ1678;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.feeder.Feeder;
import org.frogforce503.robot.subsystems.indexer.Indexer;
import org.frogforce503.robot.subsystems.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.launcher.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.launcher.hood.Hood;
import org.frogforce503.robot.viz.GameViz;
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
    private final GameViz gameViz;
    @Getter private final FollowPath.Builder blineAutoBuilder;

    private final LoggedDashboardChooser<AutoMode> routineChooser = new LoggedDashboardChooser<>("Auto");

    private Command autoCommand;
    private AutoMode lastSelectedAuto;

    private final double simAutoTimeSec = 20;

    public AutoChooser(
        Drive drive,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Indexer indexer,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        Climber climber,
        GameViz gameViz
    ) {
        this.drive = drive;
        this.gameViz = gameViz;

        // Configure PathPlanner
        PathPlannerUtil.configureAutoBuilder(drive);
        Pathfinding.setPathfinder(new LocalADStarAK());

        // Configure BLine
        blineAutoBuilder = BLineUtil.configureAutoBuilder(drive);

        // Configure autos
        routineChooser.addOption(
            "Left 2 NZ 1678",
            new Left2NZ1678(drive, intakePivot, intakeRoller, indexer, feeder, hood, flywheels, climber, gameViz, blineAutoBuilder));

        routineChooser.addOption(
            "Right 2 NZ 1678",
            new Right2NZ1678(drive, intakePivot, intakeRoller, indexer, feeder, hood, flywheels, climber, gameViz, blineAutoBuilder));

        routineChooser.addOption(
            "Preload + Depot + Climb",
            new PreloadDepotClimb(drive, intakePivot, intakeRoller, indexer, feeder, hood, flywheels, climber, gameViz, blineAutoBuilder));
    }

    public void startAuto() {
        AutoMode selectedAuto = routineChooser.get();

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

        CommandScheduler.getInstance().schedule(autoCommand);
    }

    public void updateAutoSelection() {
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

    public void cancelAuto() {
        logTrajectory(); // Clear poses

        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    private void logTrajectory(Pose2d... trajectory) {
        gameViz.getField2d().getObject("Trajectory").setPoses(trajectory);
        Logger.recordOutput("Drive/Trajectory", trajectory);
    }
}