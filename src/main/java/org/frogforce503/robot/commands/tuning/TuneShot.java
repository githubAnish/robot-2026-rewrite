package org.frogforce503.robot.commands.tuning;

import java.util.OptionalDouble;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.rebuilt.MapleSimUtil;
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
import edu.wpi.first.wpilibj.RobotBase;
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
        ShotInfo shotInfo =
            ShotCalculator.calculateHubShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());
                
        TurretSetpoint turretSetpoint =
            ShotCalculator.calculateTurretRobotRelativeSetpoint(
                shotInfo.turretFieldRelativeAngle(),
                shotInfo.turretVelocityRadPerSec(),
                drive.getAngle(),
                drive.getRobotVelocity().omegaRadiansPerSecond);

        turret.setAngle(turretSetpoint.angleRad(), turretSetpoint.velocityRadPerSec());
        flywheels.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(flywheelsVelocityRpm.get()));
        hood.setAngle(Units.degreesToRadians(hoodAngleDeg.get()), 0.0);

        if (RobotBase.isSimulation()) {
            MapleSimUtil.scoreFuelIntoHub(
                drive.getPose(),
                drive.getFieldVelocity(),
                shotInfo.turretFieldRelativeAngle(),
                shotInfo.hoodAngleRad(),
                shotInfo.flywheelsVelocityRadPerSec());
        }

        if (recordShot.get()) {
            String info = "Distance: " + shotInfo.turretToTargetDistance() + " m, " +
                          "Flywheels Velocity: " + flywheelsVelocityRpm.get() + " rpm, " +
                          "Hood Angle: " + hoodAngleDeg.get() + " deg, " +
                          "Time of Flight: " + timeOfFlightSec.get() + " sec";

            Logger.recordOutput("TuneShot/Latest Parameters", info);

            recordShot.set(false);
        }

        Logger.recordOutput("TuneShot/ShotInfo", shotInfo);
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