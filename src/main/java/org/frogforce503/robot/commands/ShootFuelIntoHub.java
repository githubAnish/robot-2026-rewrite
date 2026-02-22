package org.frogforce503.robot.commands;

import org.frogforce503.lib.rebuilt.MapleSimUtil;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotPreset;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.TurretSetpoint;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootFuelIntoHub extends Command {
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

    public ShootFuelIntoHub(Drive drive, Vision vision, Superstructure superstructure) {
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

        switch (shotPreset) {
            case NONE:
                ShotInfo shotInfo =
                    ShotCalculator.calculateHubShotInfo(
                        drive.getPose(),
                        drive.getRobotVelocity(),
                        drive.getFieldVelocity());

                turretFieldRelativeAngle = shotInfo.turretFieldRelativeAngle();
                turretVelocityRadPerSec = shotInfo.turretVelocityRadPerSec();
                flywheelsVelocityRadPerSec = shotInfo.flywheelsVelocityRadPerSec();
                hoodAngleRad = shotInfo.hoodAngleRad();
                hoodVelocityRadPerSec = shotInfo.hoodVelocityRadPerSec();
                isFeasibleShot = shotInfo.isFeasibleShot();
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

        TurretSetpoint turretSetpoint =
            ShotCalculator.calculateTurretRobotRelativeSetpoint(
                turretFieldRelativeAngle,
                turretVelocityRadPerSec,
                drive.getAngle(),
                drive.getRobotVelocity().omegaRadiansPerSecond);

        turret.setAngle(turretSetpoint.angleRad(), turretSetpoint.velocityRadPerSec());
        flywheels.setVelocity(flywheelsVelocityRadPerSec);
        hood.setAngle(hoodAngleRad, hoodVelocityRadPerSec);

        // run the feeder and indexer when shooting and find out how to shoot

        superstructure.setFeasibleShot(isFeasibleShot);

        if (RobotBase.isSimulation()) {
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
