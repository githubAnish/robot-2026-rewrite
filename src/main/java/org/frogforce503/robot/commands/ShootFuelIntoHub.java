package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.rebuilt.MapleSimUtil;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
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
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

    private final BooleanSupplier autoAssistEnabled;

    private final double kShotFireRateBallsPerSec = 10; // How many balls can you fire within 1 sec?

    public ShootFuelIntoHub(Drive drive, Vision vision, Superstructure superstructure, BooleanSupplier autoAssistEnabled) {
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

        this.autoAssistEnabled = autoAssistEnabled;

        addRequirements(intakePivot, intakeRoller, indexer, feeder, turret, flywheels, hood);
    }

    @Override
    public void initialize() {

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
            shotInfo.turretAngle(),
            shotInfo.turretVelocity(),
            drive.getAngle(),
            drive.getRobotVelocity().omegaRadiansPerSecond);

        turret.setAngle(setpoint.angleRad(), setpoint.velocityRadPerSec());

        if (RobotBase.isSimulation()) {
            MapleSimUtil.scoreFuelIntoHub(
                drive.getPose(),
                drive.getFieldVelocity(),
                shotInfo.turretAngle(),
                Units.degreesToRadians(80),
                kShotFireRateBallsPerSec);
        }

        Logger.recordOutput("ShootFuelIntoHub/ShotInfo", shotInfo);
        Logger.recordOutput("ShootFuelIntoHub/TurretSetpoint", setpoint);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
