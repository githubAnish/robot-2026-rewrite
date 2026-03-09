package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.ShotPreset;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.Getter;
import lombok.Setter;

public class TrackTargetCommand extends Command {
    private final Drive drive;
    private final Vision vision;
    private final Turret turret;
    private final Hood hood;
    private final Flywheels flywheels;

    private final BooleanSupplier isShootingSupplier;

    // State
    @Setter private ShotPreset shotPreset = ShotPreset.NONE;
    @Getter private boolean isShotFeasible = false;

    public TrackTargetCommand(
        Drive drive,
        Vision vision,
        Turret turret,
        Hood hood,
        Flywheels flywheels,
        BooleanSupplier isShootingSupplier
    ) {
        this.drive = drive;
        this.vision = vision;
        this.turret = turret;
        this.hood = hood;
        this.flywheels = flywheels;

        this.isShootingSupplier = isShootingSupplier;
        
        addRequirements(turret, hood, flywheels);
    }

    @Override
    public void initialize() {
        vision.setDesiredAprilTagGoal(
            FieldConstants.inAllianceZone(drive.getPose()) // check if should aim at hub
                ? AprilTagGoal.TURRET_HUB_AIMING
                : AprilTagGoal.GLOBAL_LOCALIZATION);
    }

    @Override
    public void execute() {
        final boolean trackingHub = FieldConstants.inAllianceZone(drive.getPose());
        final boolean underTrench = FieldConstants.Trench.contains(drive.getPose().getTranslation());

        // Define shot params
        Rotation2d turretFieldRelativeAngle = Rotation2d.kZero;
        double turretVelocityRadPerSec = 0.0;
        double hoodAngleRad = 0.0;
        double hoodVelocityRadPerSec = 0.0;
        double flywheelsVelocityRadPerSec = 0.0;

        // Calculate shot params
        switch (shotPreset) {
            case NONE:
                ShotInfo shotInfo =
                    ShotCalculator.calculateShotInfo(
                        drive.getPose(),
                        drive.getRobotVelocity(),
                        drive.getFieldVelocity());

                turretFieldRelativeAngle = shotInfo.turretFieldRelativeAngle();
                turretVelocityRadPerSec = shotInfo.turretVelocityRadPerSec();
                hoodAngleRad = shotInfo.hoodAngleRad();
                hoodVelocityRadPerSec = shotInfo.hoodVelocityRadPerSec();
                flywheelsVelocityRadPerSec = shotInfo.flywheelsVelocityRadPerSec();
                isShotFeasible =
                    trackingHub
                        ? MathUtils.inRange(shotInfo.turretToTargetDistance(), ShotCalculator.minDistanceHubShoot, ShotCalculator.maxDistanceHubShoot)
                        : MathUtils.inRange(shotInfo.turretToTargetDistance(), ShotCalculator.minDistanceLobShoot, ShotCalculator.maxDistanceLobShoot);
                break;

            case BATTER:
                turretFieldRelativeAngle = TurretConstants.BATTER_FIELD_RELATIVE;
                turretVelocityRadPerSec = 0.0;
                hoodAngleRad = HoodConstants.BATTER;
                hoodVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.BATTER;
                isShotFeasible = true;
                break;

            case TRENCH:
                turretFieldRelativeAngle = TurretConstants.TRENCH_FIELD_RELATIVE;
                turretVelocityRadPerSec = 0.0;
                hoodAngleRad = HoodConstants.TRENCH;
                hoodVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.TRENCH;
                isShotFeasible = true;
                break;

            case DEPOT:
                turretFieldRelativeAngle = TurretConstants.DEPOT_FIELD_RELATIVE;
                turretVelocityRadPerSec = 0.0;
                hoodAngleRad = HoodConstants.DEPOT;
                hoodVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.DEPOT;
                isShotFeasible = true;
                break;
        }

        if (underTrench) { // Hood ducks under trench
            hoodAngleRad = HoodConstants.DUCK_UNDER_TRENCH;
            hoodVelocityRadPerSec = 0.0;
        }

        if (!isShootingSupplier.getAsBoolean()) { // Flywheels idle if not shooting
            flywheels.setVelocity(FlywheelsConstants.IDLE);
        }

        // Run subsystems
        turret.setFieldRelativeAngle(turretFieldRelativeAngle, turretVelocityRadPerSec);
        hood.setAngle(hoodAngleRad, hoodVelocityRadPerSec);
        flywheels.setVelocity(flywheelsVelocityRadPerSec);

        // Check if subsystems at setpoint
        boolean turretAtGoal = turret.isAtAngle(turretFieldRelativeAngle, TurretConstants.kShootOnMoveTolerance);
        boolean hoodAtGoal = hood.isAtAngle(hoodAngleRad, HoodConstants.kShootOnMoveTolerance);
        boolean flywheelsAtGoal = flywheels.isAtVelocity(flywheelsVelocityRadPerSec, FlywheelsConstants.kTolerance);

        isShotFeasible = isShotFeasible && turretAtGoal && hoodAtGoal && flywheelsAtGoal;

        // Log data
        Logger.recordOutput("TrackTargetCommand/Tracking Hub?", trackingHub);
        Logger.recordOutput("TrackTargetCommand/Under Trench?", underTrench);

        Logger.recordOutput("TrackTargetCommand/Turret At Goal?", turretAtGoal);
        Logger.recordOutput("TrackTargetCommand/Hood At Goal?", hoodAtGoal);
        Logger.recordOutput("TrackTargetCommand/Flywheels At Goal?", flywheelsAtGoal);
        Logger.recordOutput("TrackTargetCommand/Is Shot Feasible?", isShotFeasible);
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
        flywheels.stop();
    }
}