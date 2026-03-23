package org.frogforce503.robot.commands;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private final double trenchDuckLookaheadSec = 0.5;

    @Getter private boolean trackingHub = false;
    @Getter private double turretToTargetDistance = 0.0;
    @Getter private boolean isShotFeasible = false;

    @Setter private OptionalDouble hoodAngleOverride = OptionalDouble.empty();
    @Setter private OptionalDouble flywheelsVelocityOverride = OptionalDouble.empty();

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
    public void initialize() {}

    @Override
    public void execute() {
        // Get inputs
        final Pose2d robotPose = drive.getPose();
        final ChassisSpeeds robotRelativeVelocity = drive.getRobotVelocity();
        final ChassisSpeeds fieldRelativeVelocity = drive.getFieldVelocity();

        trackingHub = FieldConstants.inAllianceZone(robotPose);
        final boolean underTrench = FieldConstants.Trench.contains(robotPose, fieldRelativeVelocity, trenchDuckLookaheadSec);

        // Set vision goal
        vision.setDesiredAprilTagGoal(trackingHub ? AprilTagGoal.TURRET_HUB_AIMING : AprilTagGoal.GLOBAL_LOCALIZATION);

        // Calculate shot parameters
        ShotInfo shotInfo =
            ShotCalculator.calculateShotInfo(
                robotPose,
                robotRelativeVelocity,
                fieldRelativeVelocity);

        Rotation2d turretFieldRelativeAngle = shotInfo.turretFieldRelativeAngle();
        double turretVelocityRadPerSec = shotInfo.turretVelocityRadPerSec();

        double hoodAngleRad = hoodAngleOverride.isPresent() ? hoodAngleOverride.getAsDouble() : shotInfo.hoodAngleRad();
        double hoodVelocityRadPerSec = hoodAngleOverride.isPresent() ? 0.0 : shotInfo.hoodVelocityRadPerSec();
        double flywheelsVelocityRadPerSec = flywheelsVelocityOverride.isPresent() ? flywheelsVelocityOverride.getAsDouble() : shotInfo.flywheelsVelocityRadPerSec();

        turretToTargetDistance = shotInfo.turretToTargetDistance();

        isShotFeasible =
            trackingHub
                ? MathUtils.inRange(turretToTargetDistance, ShotCalculator.minDistanceHubShoot, ShotCalculator.maxDistanceHubShoot)
                : MathUtils.inRange(turretToTargetDistance, ShotCalculator.minDistanceLobShoot, ShotCalculator.maxDistanceLobShoot);

        if (underTrench) {
            hoodAngleRad = HoodConstants.DUCK_UNDER_TRENCH;
            hoodVelocityRadPerSec = 0.0;
            isShotFeasible = false; // Don't shoot under trench
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