package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.feeder.Feeder;
import org.frogforce503.robot.subsystems.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.indexer.Indexer;
import org.frogforce503.robot.subsystems.indexer.IndexerConstants;
import org.frogforce503.robot.subsystems.launcher.LaunchCalculator;
import org.frogforce503.robot.subsystems.launcher.LaunchCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.launcher.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.launcher.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.launcher.hood.Hood;
import org.frogforce503.robot.subsystems.launcher.hood.HoodConstants;
import org.frogforce503.robot.viz.GameViz;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

/** Tunes hood angle and flywheels speed for a specific distance or preset. */
public class TuneShot extends Command {
    private final Drive drive;
    private final Indexer indexer;
    private final Feeder feeder;
    private final Hood hood;
    private final Flywheels flywheels;
    private final GameViz gameViz;

    private final LoggedTunableNumber hoodAngleDeg =
        new LoggedTunableNumber(
            "TuneShot/HoodAngleDeg",
            Units.radiansToDegrees(HoodConstants.START));

    private final LoggedTunableNumber flywheelsVelocityRpm =
        new LoggedTunableNumber(
            "TuneShot/FlywheelsVelocityRpm",
            Units.radiansPerSecondToRotationsPerMinute(FlywheelsConstants.START));

    private final LoggedNetworkBoolean recordShot =
        new LoggedNetworkBoolean(
            "Tuning/TuneShot/Record Shot?",
            false);

    private final LoggedNetworkBoolean shootInSim =
        new LoggedNetworkBoolean(
            "Tuning/TuneShot/Shoot in Sim?",
            false);

    private String hoodMapEntries = "";
    private String flywheelsMapEntries = "";

    public TuneShot(
        Drive drive,
        Indexer indexer,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        GameViz gameViz
    ) {
        this.drive = drive;
        this.indexer = indexer;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheels = flywheels;
        this.gameViz = gameViz;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Get latest shot info
        ShotInfo shotInfo =
            LaunchCalculator.getInstance().calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());

        // Run subsystems
        indexer.setVelocity(IndexerConstants.SHOOT);
        feeder.setVelocity(FeederConstants.SHOOT);
        hood.setAngle(Units.degreesToRadians(hoodAngleDeg.get()), 0.0);
        flywheels.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(flywheelsVelocityRpm.get()));

        // Simulate shooting
        if (RobotBase.isSimulation() && shootInSim.getAsBoolean()) {
            gameViz.shootFuel();
        }

        // Record shot
        if (recordShot.get()) {
            recordShot(shotInfo.launcherToTargetDistance());

            recordShot.set(false);
        }
    }

    private void recordShot(double launcherToTargetDistance) {
        final String prefix =
            LaunchCalculator.inAllianceZone(drive.getPose()) ? "hub" : "lob";

        hoodMapEntries +=
            prefix + "HoodAngleMap.put(" + launcherToTargetDistance + ", Units.degreesToRadians(" + hoodAngleDeg.get() + "));\n";

        flywheelsMapEntries +=
            prefix + "FlywheelSpeedMap.put(" + launcherToTargetDistance + ", Units.rotationsPerMinuteToRadiansPerSecond(" + flywheelsVelocityRpm.get() + "));\n";

        Logger.recordOutput("TuneShot/Shot Info (for map)", hoodMapEntries + "\n" + flywheelsMapEntries);
    }
}