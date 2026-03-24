package org.frogforce503.robot.auto.autos;

import org.frogforce503.lib.auto.bline.BLineUtil;
import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.auto.AutoMode;
import org.frogforce503.robot.commands.IntakeFuelFromGround;
import org.frogforce503.robot.commands.ShootFuelIntoHubOrLob;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class ShootPreloadNZOnce implements AutoMode {
    private final Drive drive;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Feeder feeder;
    private final Hood hood;
    private final Flywheels flywheels;
    private final GameViz gameViz;
    private final FollowPath.Builder autoBuilder;

    private final Path path;

    public ShootPreloadNZOnce(
        Drive drive,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        GameViz gameViz,
        FollowPath.Builder autoBuilder
    ) {
        this.drive = drive;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheels = flywheels;
        this.gameViz = gameViz;
        this.autoBuilder = autoBuilder;

        path = new Path("FirstTimeToNZ");
    }

    @Override
    public Command getCommand() {
        return Commands.sequence(
            new ShootFuelIntoHubOrLob(drive, feeder, hood, flywheels, gameViz).withTimeout(3), // shoot preload
            Commands.deadline(
                autoBuilder.build(path),
                Commands.waitSeconds(1).andThen(new IntakeFuelFromGround(intakePivot, intakeRoller, gameViz)) // first intake from NZ
            ),
            new ShootFuelIntoHubOrLob(drive, feeder, hood, flywheels, gameViz)
        );
    }

    @Override
    public Pose2d[] getPoses() {
        if (FieldConstants.isRed()) {
            path.flip();
        }
        return BLineUtil.getPoses(path);
    }
}