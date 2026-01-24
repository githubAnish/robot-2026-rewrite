package org.frogforce503.robot;

import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.vision.VisionSimulator;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;

/** Simulates the field, including interaction with & movement of game elements. Implement physics simulation here. */
public class GameViz {
    private final Drive drive;
    private final VisionSimulator visionViz;
    private final SimulatedArena arena;
    
    public GameViz(Drive drive, VisionSimulator visionViz) {
        this.drive = drive;
        this.visionViz = visionViz;
        arena = SimulatedArena.getInstance();
    }

    public void update() {
        visionViz.update(drive.getPose());

        Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
        Logger.recordOutput("GameViz/FuelPoses", fuelPoses);
    }
}