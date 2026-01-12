package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.FlywheelsConfig;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneFlywheels extends Command {
    private final Flywheels flywheels;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber setpointVelocityRpm;

    public TuneFlywheels(Flywheels flywheels) {
        this.flywheels = flywheels;

        // Get initial values from config
        final FlywheelsConfig flywheelsConfig = Robot.bot.getFlywheelsConfig();

        final PIDConfig initialPID = flywheelsConfig.kPID();
        final FFConfig initialFF = flywheelsConfig.kFF();

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("Flywheels/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("Flywheels/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("Flywheels/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("Flywheels/kS", initialFF.kS());
        this.kV = new LoggedTunableNumber("Flywheels/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("Flywheels/kA", initialFF.kA());

        this.setpointVelocityRpm = new LoggedTunableNumber("Flywheels/SetpointRpm", Units.radiansPerSecondToRotationsPerMinute(FlywheelsConstants.START));

        addRequirements(flywheels);
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
            () -> flywheels.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> flywheels.setFeedforward(new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get())),
            kS, kV, kA);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> flywheels.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(setpointVelocityRpm.get())),
            setpointVelocityRpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
    }
}