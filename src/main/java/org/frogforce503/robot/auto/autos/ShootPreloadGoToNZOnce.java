package org.frogforce503.robot.auto.autos;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.frogforce503.lib.auto.AutoUtil;
import org.frogforce503.lib.auto.pathplanner.PathPlannerUtil;
import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.auto.AutoMode;
import org.frogforce503.robot.commands.IntakeFuelFromGround;
import org.frogforce503.robot.commands.ShootFuelIntoHubOrLob;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.vision.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShootPreloadGoToNZOnce implements AutoMode {
    private final Drive drive;
    private final Vision vision;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Feeder feeder;
    private final GameViz gameViz;

    private final BooleanSupplier isShotFeasibleSupplier;

    private final PathPlannerPath firstTimeToNZ;

    public ShootPreloadGoToNZOnce(
        Drive drive,
        Vision vision,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Feeder feeder,
        GameViz gameViz,
        BooleanSupplier isShotFeasibleSupplier
    ) {
        this.drive = drive;
        this.vision = vision;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.feeder = feeder;
        this.gameViz = gameViz;

        this.isShotFeasibleSupplier = isShotFeasibleSupplier;

        firstTimeToNZ = PathPlannerUtil.loadTrajectory("FirstTimeToNZ");
    }

    @Override
    public Command getCommand() {
        return Commands.sequence(
            new ShootFuelIntoHubOrLob(feeder, gameViz, isShotFeasibleSupplier).withTimeout(3), // shoot preload
            Commands.deadline(
                AutoBuilder.followPath(firstTimeToNZ),
                new IntakeFuelFromGround(drive, vision, intakePivot, intakeRoller, gameViz, () -> true) // first intake from NZ
            ),
            new ShootFuelIntoHubOrLob(feeder, gameViz, isShotFeasibleSupplier)
        );
    }

    @Override
    public List<Pose2d> getPoses() {
        return AutoUtil.getPoses(firstTimeToNZ);
    }
}
