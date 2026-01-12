package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj2.command.Command;

// Notes:
// shoot on move (auto aim + future pose prediction + shooting)
// Use the ShotCalculator.java to predict shots, and use the superstructure shotpresets to determine if calculating the flywheels speed, hood angle, etc is needed
public class ShootFuelIntoHub extends Command {
    private final Drive drive;
    private final Vision vision;

    private final BooleanSupplier autoAssistEnabled;

    public ShootFuelIntoHub(Drive drive, Vision vision, Superstructure superstructure, BooleanSupplier autoAssistEnabled) {
        this.drive = drive;
        this.vision = vision;
        this.autoAssistEnabled = autoAssistEnabled;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
