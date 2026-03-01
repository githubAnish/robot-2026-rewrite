package org.frogforce503.robot.commands;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.rebuilt.MapleSimUtil;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotPreset;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.TurretSetpoint;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.indexer.IndexerConstants;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class LobFuelIntoAlliance extends Command {
    private final Drive drive;
    private final Vision vision;

    private final Superstructure superstructure;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;
    private final Feeder feeder;
    private final Turret turret;
    private final Flywheels flywheels;
    private final Hood hood;

    private final boolean isShooting;

    public LobFuelIntoAlliance(Drive drive, Vision vision, Superstructure superstructure, boolean isShooting) {
        this.drive = drive;
        this.vision = vision;

        this.superstructure = superstructure;
        this.intakePivot = superstructure.getIntakePivot();
        this.intakeRoller = superstructure.getIntakeRoller();
        this.indexer = superstructure.getIndexer();
        this.feeder = superstructure.getFeeder();
        this.turret = superstructure.getTurret();
        this.flywheels = superstructure.getFlywheels();
        this.hood = superstructure.getHood();

        this.isShooting = isShooting;

        addRequirements(intakePivot, intakeRoller, indexer, feeder, turret, flywheels, hood);
    }

    @Override
    public void initialize() {
        vision.setDesiredAprilTagGoal(AprilTagGoal.TURRET_HUB_AIMING);
    }

    @Override
    public void execute() {
        Rotation2d turretFieldRelativeAngle = Rotation2d.kZero;
        double turretVelocityRadPerSec = 0.0;
        double flywheelsVelocityRadPerSec = 0.0;
        double hoodAngleRad = 0.0;
        double hoodVelocityRadPerSec = 0.0;
        boolean isFeasibleShot = false;

        ShotPreset shotPreset = superstructure.getShotPreset();

        // Calculate shot params
        switch (shotPreset) {
            case NONE:
                ShotInfo shotInfo =
                    ShotCalculator.calculateLobShotInfo(
                        drive.getPose(),
                        drive.getRobotVelocity(),
                        drive.getFieldVelocity());

                turretFieldRelativeAngle = shotInfo.turretFieldRelativeAngle();
                turretVelocityRadPerSec = shotInfo.turretVelocityRadPerSec();
                flywheelsVelocityRadPerSec = shotInfo.flywheelsVelocityRadPerSec();
                hoodAngleRad = shotInfo.hoodAngleRad();
                hoodVelocityRadPerSec = shotInfo.hoodVelocityRadPerSec();
                isFeasibleShot = MathUtils.inRange(shotInfo.turretToTargetDistance(), ShotCalculator.minDistanceToHub, ShotCalculator.maxDistanceToHub);
                break;

            case BATTER:
                turretFieldRelativeAngle = TurretConstants.BATTER_FIELD_RELATIVE;
                turretVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.BATTER;
                hoodAngleRad = HoodConstants.BATTER;
                hoodVelocityRadPerSec = 0.0;
                isFeasibleShot = true;
                break;

            case LOB_FROM_NZ:
                turretFieldRelativeAngle = TurretConstants.LOB_FROM_NZ_FIELD_RELATIVE;
                turretVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.LOB_FROM_NZ;
                hoodAngleRad = HoodConstants.LOB_FROM_NZ;
                hoodVelocityRadPerSec = 0.0;
                isFeasibleShot = true;
                break;

            case TOWER:
                turretFieldRelativeAngle = TurretConstants.TOWER_FIELD_RELATIVE;
                turretVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.TOWER;
                hoodAngleRad = HoodConstants.TOWER;
                hoodVelocityRadPerSec = 0.0;
                isFeasibleShot = true;
                break;
        }

        // Calculate robot-relative turret setpoint
        TurretSetpoint turretSetpoint =
            ShotCalculator.calculateTurretRobotRelativeSetpoint(
                turretFieldRelativeAngle,
                turretVelocityRadPerSec,
                drive.getAngle(),
                drive.getRobotVelocity().omegaRadiansPerSecond);

        // Set shooter setpoints
        turret.setAngle(turretSetpoint.angleRad(), turretSetpoint.velocityRadPerSec());
        flywheels.setVelocity(flywheelsVelocityRadPerSec);
        hood.setAngle(hoodAngleRad, hoodVelocityRadPerSec);

        // Run indexer and feeder
        if (isShooting) {
            indexer.setVelocity(IndexerConstants.SHOOT);
            feeder.setVelocity(FeederConstants.SHOOT);
        }

        // Check if subsystems at setpoint
        boolean turretAtGoal = turret.isAtAngle(turretSetpoint.angleRad(), TurretConstants.kFixedTolerance);
        boolean flywheelsAtGoal = flywheels.isAtVelocity(flywheelsVelocityRadPerSec, FlywheelsConstants.kTolerance);
        boolean hoodAtGoal = hood.isAtAngle(hoodAngleRad, HoodConstants.kFixedTolerance);

        isFeasibleShot = isFeasibleShot && turretAtGoal && flywheelsAtGoal && hoodAtGoal;

        superstructure.setFeasibleShot(isFeasibleShot);

        // Simulated shooting
        if (RobotBase.isSimulation() && isFeasibleShot) {
            MapleSimUtil.scoreFuelIntoHub(
                drive.getPose(),
                drive.getFieldVelocity(),
                turretFieldRelativeAngle,
                hoodAngleRad,
                flywheelsVelocityRadPerSec);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        vision.setDesiredAprilTagGoal(AprilTagGoal.GLOBAL_LOCALIZATION);
    }
}
