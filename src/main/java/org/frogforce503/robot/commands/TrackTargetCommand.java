package org.frogforce503.robot.commands;

import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.ShotPreset;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TrackTargetCommand extends Command {
    private final Drive drive;
    private final Vision vision;

    private final Turret turret;
    private final Hood hood;

    public TrackTargetCommand(Drive drive, Vision vision, Turret turret, Hood hood) {
        this.drive = drive;
        this.vision = vision;

        this.turret = turret;
        this.hood = hood;
        
        addRequirements(turret, hood);
    }

    @Override
    public void initialize() {
        vision.setDesiredAprilTagGoal(
            FieldConstants.inAllianceZone(drive.getPose())
                ? AprilTagGoal.TURRET_HUB_AIMING
                : AprilTagGoal.GLOBAL_LOCALIZATION);
    }

    @Override
    public void execute() {
        final boolean trackingHub = FieldConstants.inAllianceZone(drive.getPose()); // For checking whether to compute hub or lob shot info
        final boolean underTrench = FieldConstants.underTrench(drive.getPose());

        // Define shot params
        Rotation2d turretFieldRelativeAngle = Rotation2d.kZero;
        double turretVelocityRadPerSec = 0.0;
        double hoodAngleRad = 0.0;
        double hoodVelocityRadPerSec = 0.0;

        final ShotPreset shotPreset = ShotCalculator.getInstance().getShotPreset();

        // Calculate shot params
        switch (shotPreset) {
            case NONE:
                ShotInfo shotInfo =
                    trackingHub
                        ? ShotCalculator.getInstance().calculateHubShotInfo(
                            drive.getPose(),
                            drive.getRobotVelocity(),
                            drive.getFieldVelocity())

                        : ShotCalculator.getInstance().calculateLobShotInfo(
                            drive.getPose(),
                            drive.getRobotVelocity(),
                            drive.getFieldVelocity());

                turretFieldRelativeAngle = shotInfo.turretFieldRelativeAngle();
                turretVelocityRadPerSec = shotInfo.turretVelocityRadPerSec();
                hoodAngleRad = shotInfo.hoodAngleRad();
                hoodVelocityRadPerSec = shotInfo.hoodVelocityRadPerSec();
                break;

            case BATTER:
                turretFieldRelativeAngle = TurretConstants.BATTER_FIELD_RELATIVE;
                turretVelocityRadPerSec = 0.0;
                hoodAngleRad = HoodConstants.BATTER;
                hoodVelocityRadPerSec = 0.0;
                break;

            case TRENCH:
                turretFieldRelativeAngle = TurretConstants.TRENCH_FIELD_RELATIVE;
                turretVelocityRadPerSec = 0.0;
                hoodAngleRad = HoodConstants.TRENCH;
                hoodVelocityRadPerSec = 0.0;
                break;

            case DEPOT:
                turretFieldRelativeAngle = TurretConstants.DEPOT_FIELD_RELATIVE;
                turretVelocityRadPerSec = 0.0;
                hoodAngleRad = HoodConstants.DEPOT;
                hoodVelocityRadPerSec = 0.0;
                break;
        }

        if (underTrench) {
            hoodAngleRad = HoodConstants.DUCK_UNDER_TRENCH;
            hoodVelocityRadPerSec = 0.0;
        }

        // Set shooter setpoints
        turret.setFieldRelativeAngle(turretFieldRelativeAngle, turretVelocityRadPerSec);
        hood.setAngle(hoodAngleRad, hoodVelocityRadPerSec);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        vision.setDesiredAprilTagGoal(AprilTagGoal.GLOBAL_LOCALIZATION);

        turret.stop();
        hood.stop();
    }
}