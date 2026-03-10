package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to tune hood angle and flywheels speed for a specific distance or preset. */
public class TuneShot extends Command {
    private final Drive drive;
    private final Turret turret;
    private final Hood hood;
    private final Flywheels flywheels;
    private final GameViz gameViz;

    private final boolean tuningHubShot;

    private final LoggedTunableNumber hoodAngleDeg =
        new LoggedTunableNumber("TuneShot/HoodAngleDeg", Units.radiansToDegrees(HoodConstants.START));

    private final LoggedTunableNumber flywheelsVelocityRpm =
        new LoggedTunableNumber("TuneShot/FlywheelsVelocityRpm", Units.radiansPerSecondToRotationsPerMinute(FlywheelsConstants.START));

    private final LoggedNetworkBoolean recordShot =
        new LoggedNetworkBoolean("Tuning/TuneShot/Record Shot?", false);

    public TuneShot(
        Drive drive,
        Turret turret,
        Hood hood,
        Flywheels flywheels,
        GameViz gameViz,
        boolean tuningHubShot
    ) {
        this.drive = drive;
        this.turret = turret;
        this.hood = hood;
        this.flywheels = flywheels;
        this.gameViz = gameViz;

        this.tuningHubShot = tuningHubShot;

        addRequirements(turret, hood, flywheels);
    }

    @Override
    public void initialize() {
        flywheelsVelocityRpm.setTuningMode(true);
        hoodAngleDeg.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Define shot params
        Rotation2d turretFieldRelativeAngle = Rotation2d.kZero;
        double turretVelocityRadPerSec = 0.0;
        double flywheelsVelocityRadPerSec = 0.0;
        double hoodAngleRad = 0.0;

        // Calculate shot params
        ShotInfo shotInfo =
            ShotCalculator.calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());

        turretFieldRelativeAngle = shotInfo.turretFieldRelativeAngle();
        turretVelocityRadPerSec = shotInfo.turretVelocityRadPerSec();
        flywheelsVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(flywheelsVelocityRpm.get());
        hoodAngleRad = Units.degreesToRadians(hoodAngleDeg.get());

        // Run subsystems
        turret.setFieldRelativeAngle(turretFieldRelativeAngle, turretVelocityRadPerSec);
        flywheels.setVelocity(flywheelsVelocityRadPerSec);
        hood.setAngle(hoodAngleRad);

        // Simulate shooting
        if (RobotBase.isSimulation()) {
            gameViz.shootFuel(false);
        }

        if (recordShot.get()) {
            String info = "Distance: " + shotInfo.turretToTargetDistance() + " m, " +
                          "Flywheels Velocity: " + flywheelsVelocityRpm.get() + " rpm, " +
                          "Hood Angle: " + hoodAngleDeg.get() + " deg";

            Logger.recordOutput("TuneShot/Latest Parameters", info);

            recordShot.set(false);
        }

        // Log data
        Logger.recordOutput("TuneShot/Tuning Hub Shot?", tuningHubShot);
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