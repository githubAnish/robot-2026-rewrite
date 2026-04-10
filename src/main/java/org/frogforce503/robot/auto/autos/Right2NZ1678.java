package org.frogforce503.robot.auto.autos;

import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.viz.GameViz;

import frc.robot.lib.BLine.FollowPath;

public class Right2NZ1678 extends Left2NZ1678 {
    public Right2NZ1678(
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

        firstTimeToNZAndBack.mirror();
        secondTimeToNZAndBack.mirror();
    }
}