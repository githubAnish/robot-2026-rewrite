package org.frogforce503.robot;

import java.util.Arrays;

import org.frogforce503.lib.rebuilt.MapleSimUtil;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.SuperstructureViz;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.vision.VisionSimulator;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.Getter;

/** Simulates the field, including interaction with & movement of game elements. Implement physics simulation here. */
public class GameViz {
    private final Drive drive;
    private final Turret turret;
    private final Hood hood;
    private final IntakePivot intakePivot;

    private final VisionSimulator visionViz;
    private final SuperstructureViz superstructureViz;

    @Getter private final IntakeSimulation intakeSimulation;
    
    public GameViz(Drive drive, Turret turret, Hood hood, IntakePivot intakePivot, VisionSimulator visionViz) {
        this.drive = drive;

        this.turret = turret;
        this.hood = hood;
        this.intakePivot = intakePivot;

        this.visionViz = visionViz;
        this.superstructureViz = new SuperstructureViz();

        this.intakeSimulation = MapleSimUtil.createIntake(drive.getMapleSimDrive().mapleSimDrive);

        resetFieldForAuto();
    }

    public void resetFieldForAuto() {
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void update() {
        visionViz.update(drive.getPose());
        
        superstructureViz.update(
            new Pose3d(drive.getPose()),
            turret.getRobotRelativeAngleRad(),
            hood.getAngleRad(),
            intakePivot.getAngleRad());

        Translation3d[] fuelTranslations = // Convert fuel poses to translations to lower data processed by NetworkTables
            Arrays
                .stream(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")) // Get all fuel from MapleSim arena
                .map(Pose3d::getTranslation)
                .toArray(Translation3d[]::new);

        Logger.recordOutput("GameViz/FuelTranslations", fuelTranslations);
        Logger.recordOutput("GameViz/NumFuelInRobot", intakeSimulation.getGamePiecesAmount());
    }
}