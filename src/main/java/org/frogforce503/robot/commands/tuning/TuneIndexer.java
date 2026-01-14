package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.IndexerConfig;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.indexer.IndexerConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneIndexer extends Command {
    private final Indexer indexer;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber setpointVelocityRpm;

    public TuneIndexer(Indexer indexer) {
        this.indexer = indexer;

        // Get initial values from config
        final IndexerConfig indexerConfig = Robot.bot.getIndexerConfig();

        final PIDConfig initialPID = indexerConfig.kPID();
        final FFConfig initialFF = indexerConfig.kFF();

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("Indexer/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("Indexer/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("Indexer/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("Indexer/kS", initialFF.kS());
        this.kV = new LoggedTunableNumber("Indexer/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("Indexer/kA", initialFF.kA());

        this.setpointVelocityRpm = new LoggedTunableNumber("Indexer/SetpointRpm", Units.radiansPerSecondToRotationsPerMinute(IndexerConstants.START));

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        // Set tuning mode to true
        this.kP.setTuningMode(true);
        this.kI.setTuningMode(true);
        this.kD.setTuningMode(true);
        this.kS.setTuningMode(true);
        this.kV.setTuningMode(true);
        this.kA.setTuningMode(true);
        this.setpointVelocityRpm.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Update PID only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> indexer.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> indexer.setFeedforward(new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get())),
            kS, kV, kA);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> indexer.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(setpointVelocityRpm.get())),
            setpointVelocityRpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }
}