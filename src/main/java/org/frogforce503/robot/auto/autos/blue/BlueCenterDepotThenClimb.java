package org.frogforce503.robot.auto.autos.blue;

import java.util.ArrayList;
import java.util.List;

import org.frogforce503.robot.auto.AutoMode;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.vision.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlueCenterDepotThenClimb implements AutoMode {
    public BlueCenterDepotThenClimb(Drive drive, Vision vision, Superstructure superstructure) {
            
    }

    @Override
    public Command getCommand() {
        return Commands.none();
    }

    @Override
    public List<Pose2d> getPoses() {
        return new ArrayList<>();
    }
}
