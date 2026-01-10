package org.frogforce503.robot;

import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.vision.VisionSimulator;

/** Simulates the field, including interaction with & movement of game elements. Implement physics simulation here. */
public class GameViz {
    private final Drive drive;
    private final VisionSimulator visionViz;
    
    public GameViz(Drive drive, VisionSimulator visionViz) {
        this.drive = drive;
        this.visionViz = visionViz;
    }

    public void update() {
        visionViz.update(drive.getPose());
    }
}