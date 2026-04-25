package org.frogforce503.robot.auto;

import org.frogforce503.robot.commands.AimAndPrepShot;
import org.frogforce503.robot.commands.AlignToClimb;
import org.frogforce503.robot.commands.IntakeFuelFromGround;
import org.frogforce503.robot.commands.LowerClimber;
import org.frogforce503.robot.commands.PutIntakeUpForShoot;
import org.frogforce503.robot.commands.RaiseClimber;
import org.frogforce503.robot.commands.ShootFuel;
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
    private final FollowPath.Builder blineAutoBuilder;

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
        FollowPath.Builder blineAutoBuilder
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
        this.blineAutoBuilder = blineAutoBuilder;
    }

    protected abstract Command getCommand();
    protected abstract Pose2d[] getPoses();

    protected Command drive(Path path) {
        return blineAutoBuilder.build(path);
    }

    protected Command intake() {
        return new IntakeFuelFromGround(intakePivot, intakeRoller, gameViz);
    }

    protected Command shoot() {
        return
            Commands.parallel(
                new AimAndPrepShot(drive, feeder, hood, flywheels),
                new ShootFuel(indexer, gameViz),
                new PutIntakeUpForShoot(intakePivot, intakeRoller, gameViz));
    }

    protected Command autoClimb() {
        return
            Commands.sequence(
                new RaiseClimber(climber),
                new AlignToClimb(drive),
                new LowerClimber(climber, gameViz));
    }
}