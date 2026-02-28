package org.frogforce503.robot;

import java.util.Arrays;

import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.SuperstructureViz;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.vision.VisionSimulator;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Simulates the field, including interaction with & movement of game elements. Implement physics simulation here. */
public class GameViz {
    private final Drive drive;
    private final Turret turret;
    private final Hood hood;
    private final IntakePivot intakePivot;

    private final VisionSimulator visionViz;
    private final SuperstructureViz superstructureViz;
    
    public GameViz(Drive drive, Superstructure superstructure, VisionSimulator visionViz) {
        this.drive = drive;
        this.turret = superstructure.getTurret();
        this.hood = superstructure.getHood();
        this.intakePivot = superstructure.getIntakePivot();
        this.visionViz = visionViz;
        this.superstructureViz = new SuperstructureViz();

        resetFieldForAuto();
    }

    public void resetFieldForAuto() {
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void update() {
        visionViz.update(drive.getPose());
        
        superstructureViz.update(
            new Pose3d(drive.getPose()),
            turret.getAngleRad(),
            hood.getAngleRad(),
            intakePivot.getAngleRad());

        Translation3d[] fuelTranslations = // Convert fuel poses to translations to lower data processed by NetworkTables
            Arrays
                .stream(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")) // Get all fuel from MapleSim arena
                .map(Pose3d::getTranslation)
                .toArray(Translation3d[]::new);

        Logger.recordOutput("GameViz/FuelTranslations", fuelTranslations);
    }
}