package org.frogforce503.robot.auto.autos;

import org.frogforce503.lib.auto.bline.BLineUtil;
import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.auto.AutoMode;
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

public class Left2NZ1678 extends AutoMode {
    protected final Path firstTimeToNZAndBack;
    protected final Path secondTimeToNZAndBack;

    public Left2NZ1678(
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

        firstTimeToNZAndBack = new Path("FirstTimeToNZAndBack");
        secondTimeToNZAndBack = new Path("SecondTimeToNZAndBack");
    }

    @Override
    public Command getCommand() {
        return Commands.sequence(
            Commands.deadline(
                drive(firstTimeToNZAndBack),
                intake()
            ),
            shoot().withTimeout(3.5),
            Commands.deadline(
                drive(secondTimeToNZAndBack),
                Commands.waitSeconds(1.5).andThen(intake())
            ),
            shoot()
        );
    }

    @Override
    public Pose2d[] getPoses() {
        return BLineUtil.getPoses(firstTimeToNZAndBack, secondTimeToNZAndBack);
    }
}