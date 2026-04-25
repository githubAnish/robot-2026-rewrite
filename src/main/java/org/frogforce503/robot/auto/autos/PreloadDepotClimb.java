package org.frogforce503.robot.auto.autos;

import org.frogforce503.lib.auto.bline.BLineUtil;
import org.frogforce503.robot.auto.AutoMode;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.feeder.Feeder;
import org.frogforce503.robot.subsystems.indexer.Indexer;
import org.frogforce503.robot.subsystems.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.launcher.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.launcher.hood.Hood;
import org.frogforce503.robot.viz.GameViz;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class PreloadDepotClimb extends AutoMode {
    private final Path initToDepot;
    private final Path depotToShotPose;

    public PreloadDepotClimb(
        Drive drive,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Indexer indexer,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        Climber climber,
        GameViz gameViz,
        FollowPath.Builder autoBuilder
    ) {
        super(drive, intakePivot, intakeRoller, indexer, feeder, hood, flywheels, climber, gameViz, autoBuilder);

        initToDepot = new Path("InitToDepot");
        depotToShotPose = new Path("DepotToShotPose");
    }

    @Override
    public Command getCommand() {
        return Commands.sequence(
            Commands.deadline(
                Commands.sequence(
                    drive(initToDepot),
                    drive(depotToShotPose)
                ),
                intake()
            ),
            shoot().withTimeout(5),
            autoClimb()
        );
    }

    @Override
    public Pose2d[] getPoses() {
        return BLineUtil.getPoses(initToDepot, depotToShotPose);
    }
}