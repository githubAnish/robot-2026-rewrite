package org.frogforce503.robot.auto;

import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.commands.IntakeFuelFromGround;
import org.frogforce503.robot.commands.LowerClimber;
import org.frogforce503.robot.commands.RaiseClimber;
import org.frogforce503.robot.commands.ShootFuelIntoHubOrLob;
import org.frogforce503.robot.commands.drive.AimAtHubOrLob;
import org.frogforce503.robot.commands.drive.AlignToClimb;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public abstract class AutoMode {
    private final Drive drive;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;
    private final Feeder feeder;
    private final Hood hood;
    private final Flywheels flywheels;
    private final Climber climber;
    private final GameViz gameViz;
    private final FollowPath.Builder autoBuilder;

    public AutoMode(
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
        this.drive = drive;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.indexer = indexer;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheels = flywheels;
        this.climber = climber;
        this.gameViz = gameViz;
        this.autoBuilder = autoBuilder;
    }

    protected abstract Command getCommand();
    protected abstract Pose2d[] getPoses();

    protected Command drive(Path path) {
        return autoBuilder.build(path);
    }

    protected Command intake() {
        return new IntakeFuelFromGround(intakePivot, intakeRoller, gameViz);
    }

    protected Command shoot() {
        return
            Commands.parallel(
                new AimAtHubOrLob(drive),
                new ShootFuelIntoHubOrLob(drive, indexer, feeder, hood, flywheels, gameViz));
    }

    protected Command autoClimb() {
        return
            Commands.sequence(
                new RaiseClimber(climber),
                new AlignToClimb(drive),
                new LowerClimber(climber, gameViz));
    }
}