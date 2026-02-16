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
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

// Notes:
// shoot on move (auto aim + future pose prediction + shooting)
// Use the ShotCalculator.java to predict shots, and use the superstructure shotpresets to determine if calculating the turret angle, flywheels speed, hood angle, etc is needed
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

    private final double kShotFireRateBallsPerSec = 7; // How many balls can you fire within 1 sec?

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
        // ShotInfo shotInfo =
        //     ShotCalculator.calculateHubShotInfo(
        //         drive.getPose(),
        //         drive.getRobotVelocity(),
        //         drive.getFieldVelocity());

        // if (RobotBase.isSimulation()) {
        //     MapleSimUtil.scoreFuelIntoHub(
        //         drive.getPose(),
        //         drive.getFieldVelocity(),
        //         turret.getAngleRad() + drive.getAngle().getRadians(),
        //         Units.degreesToRadians(80),
        //         kShotFireRateBallsPerSec);
        // }

        // Translation2d target =
        //     FieldConstants.isRed() 
        //         ? FieldConstants.Hub.redShotPose.toTranslation2d() 
        //         : FieldConstants.Hub.blueShotPose.toTranslation2d();
        
        // Rotation2d turretAng = target.minus(drive.getPose().plus(GeomUtil.toTransform2d(TurretConstants.robotToTurret)).getTranslation()).getAngle();

        // TurretSetpoint setpoint = ShotCalculator.calculateTurretRobotRelativeSetpoint(
        //     turretAng, 0, drive.getAngle(), drive.getRobotVelocity().omegaRadiansPerSecond);

        ShotInfo shotInfo = ShotCalculator.calculateHubShotInfo(drive.getPose(), drive.getRobotVelocity(), drive.getFieldVelocity());
                
        TurretSetpoint setpoint = ShotCalculator.calculateTurretRobotRelativeSetpoint(
            shotInfo.turretFieldRelativeAngle(),
            shotInfo.turretVelocityRadPerSec(),
            drive.getAngle(),
            drive.getRobotVelocity().omegaRadiansPerSecond);

        turret.setAngle(setpoint.angleRad(), setpoint.velocityRadPerSec());
        hood.setAngle(shotInfo.hoodAngleRad(), shotInfo.hoodVelocityRadPerSec());
        flywheels.setVelocity(shotInfo.flywheelsVelocityRadPerSec());

        superstructure.setFeasibleShot(shotInfo.isFeasibleShot());

        if (RobotBase.isSimulation()) {
            MapleSimUtil.scoreFuelIntoHub(
                drive.getPose(),
                drive.getFieldVelocity(),
                shotInfo.turretFieldRelativeAngle(),
                shotInfo.hoodAngleRad(),
                shotInfo.flywheelsVelocityRadPerSec(),
                kShotFireRateBallsPerSec);
        }

        Logger.recordOutput("ShootFuelIntoHub/ShotInfo", shotInfo);
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
