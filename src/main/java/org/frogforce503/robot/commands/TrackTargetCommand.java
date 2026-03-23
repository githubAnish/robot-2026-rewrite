package org.frogforce503.robot.commands;

import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;

import edu.wpi.first.wpilibj2.command.Command;

public class TrackTargetCommand extends Command {
    private final Drive drive;
    private final Vision vision;
    private final Turret turret;

    public TrackTargetCommand(Drive drive, Vision vision, Turret turret) {
        this.drive = drive;
        this.vision = vision;
        this.turret = turret;
        
        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Set vision goal
        vision.setDesiredAprilTagGoal(
            FieldConstants.inAllianceZone(drive.getPose())
                ? AprilTagGoal.TURRET_HUB_AIMING
                : AprilTagGoal.GLOBAL_LOCALIZATION);

        // Get latest shot info
        ShotInfo shotInfo =
            ShotCalculator.getInstance().calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());

        // Run turret
        turret.setFieldRelativeAngle(shotInfo.turretFieldRelativeAngle(), shotInfo.turretVelocityRadPerSec());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        vision.setDesiredAprilTagGoal(AprilTagGoal.GLOBAL_LOCALIZATION);
        turret.stop();
    }
}