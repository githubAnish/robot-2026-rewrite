package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.FeederConfig;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.FeederConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneFeeder extends Command {
    private final Feeder feeder;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber setpointVelocityRpm;

    public TuneFeeder(Feeder feeder) {
        this.feeder = feeder;

        // Get initial values from config
        final FeederConfig feederConfig = Robot.bot.getFeederConfig();

        final PIDConfig initialPID = feederConfig.kPID();
        final FFConfig initialFF = feederConfig.kFF();

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("Feeder/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("Feeder/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("Feeder/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("Feeder/kS", initialFF.kS());
        this.kV = new LoggedTunableNumber("Feeder/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("Feeder/kA", initialFF.kA());

        this.setpointVelocityRpm = new LoggedTunableNumber("Feeder/SetpointRpm", Units.radiansPerSecondToRotationsPerMinute(FeederConstants.START));

        addRequirements(feeder);
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
            () -> feeder.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> feeder.setFeedforward(new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get())),
            kS, kV, kA);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> feeder.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(setpointVelocityRpm.get())),
            setpointVelocityRpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}