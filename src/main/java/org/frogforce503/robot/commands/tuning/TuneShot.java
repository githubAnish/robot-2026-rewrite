package org.frogforce503.robot.commands.tuning;

import java.util.OptionalDouble;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.TurretSetpoint;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

// Notes:
// Command to tune shotmap (if there) and tune presets (batter, trench, outpost, depot, etc)
// dist to hood ang, flywheel speed, time of flight map
// write in comments that this cmd should be executed and done when robot stationary
// get dist, set to some logger.recordoutput string
public class TuneShot extends Command {
    private final Drive drive;

    private final Superstructure superstructure;
    private final Turret turret;
    private final Flywheels flywheels;
    private final Hood hood;

    private final LoggedTunableNumber flywheelsVelocityRpm =
        new LoggedTunableNumber("TuneShot/FlywheelsVelocityRpm", Units.radiansPerSecondToRotationsPerMinute(FlywheelsConstants.START));

    private final LoggedTunableNumber hoodAngleDeg =
        new LoggedTunableNumber("TuneShot/HoodAngleDeg", Units.radiansToDegrees(HoodConstants.START));

    private final LoggedTunableNumber timeOfFlightSec =
        new LoggedTunableNumber("TuneShot/TimeOfFlightSec", 0.0);

    private final LoggedNetworkBoolean recordShot =
        new LoggedNetworkBoolean("Tuning/TuneShot/Record Shot?", false);

    private ShotInfo lastShotInfo;

    public TuneShot(Drive drive, Superstructure superstructure) {
        this.drive = drive;

        this.superstructure = superstructure;
        this.turret = superstructure.getTurret();
        this.flywheels = superstructure.getFlywheels();
        this.hood = superstructure.getHood();
    }

    @Override
    public void initialize() {
        this.flywheelsVelocityRpm.setTuningMode(true);
        this.hoodAngleDeg.setTuningMode(true);
        this.timeOfFlightSec.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Get inputs
        Pose2d pose = drive.getPose();
        ChassisSpeeds robotRelativeSpeeds = drive.getRobotVelocity();
        ChassisSpeeds fieldRelativeSpeeds = drive.getFieldVelocity();

        // Calculate and apply shot parameters
        ShotInfo shotInfo =
            ShotCalculator.calculateHubShotInfo(
                pose,
                robotRelativeSpeeds,
                fieldRelativeSpeeds,
                OptionalDouble.of(Units.rotationsPerMinuteToRadiansPerSecond(flywheelsVelocityRpm.get())),
                OptionalDouble.of(Units.degreesToRadians(hoodAngleDeg.get())),
                OptionalDouble.of(timeOfFlightSec.get()));
                
        TurretSetpoint setpoint = ShotCalculator.calculateTurretRobotRelativeSetpoint(
            shotInfo.turretFieldRelativeAngle(),
            shotInfo.turretVelocityRadPerSec(),
            drive.getAngle(),
            drive.getRobotVelocity().omegaRadiansPerSecond);

        turret.setAngle(setpoint.angleRad(), setpoint.velocityRadPerSec());
        hood.setAngle(shotInfo.hoodAngleRad(), shotInfo.hoodVelocityRadPerSec());
        flywheels.setVelocity(shotInfo.flywheelsVelocityRadPerSec());

        superstructure.setFeasibleShot(shotInfo.isFeasibleShot());

        if (recordShot.get()) {
            String info = "Distance: " + shotInfo.turretToTargetDistance() + " m, " +
                          "Flywheels Velocity: " + flywheelsVelocityRpm.get() + " rpm, " +
                          "Hood Angle: " + hoodAngleDeg.get() + " deg, " +
                          "Time of Flight: " + timeOfFlightSec.get() + " sec";

            Logger.recordOutput("TuneShot/Shot Info", info);

            recordShot.set(false);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
        hood.stop();
    }   
}