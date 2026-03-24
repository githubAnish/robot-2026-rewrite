package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.climber.ClimberConstants;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneClimber extends Command {
    private final Climber climber;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kG;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber maxVelocityInchesPerSec;
    private final LoggedTunableNumber maxAccelerationInchesPerSec2;

    private final LoggedTunableNumber setpointHeightInches;

    public TuneClimber(Climber climber) {
        this.climber = climber;

        // Get initial values from config
        final PIDConfig initialPID = ClimberConstants.kPID;
        final FFConfig initialFF = ClimberConstants.kFF;
        final Constraints initialConstraints = ClimberConstants.kConstraints;

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("Climber/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("Climber/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("Climber/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("Climber/kS", initialFF.kS());
        this.kG = new LoggedTunableNumber("Climber/kG", initialFF.kG());
        this.kV = new LoggedTunableNumber("Climber/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("Climber/kA", initialFF.kA());

        this.maxVelocityInchesPerSec = new LoggedTunableNumber("Climber/MaxVelocityInchesPerSec", Units.metersToInches(initialConstraints.maxVelocity));
        this.maxAccelerationInchesPerSec2 = new LoggedTunableNumber("Climber/MaxAccelerationInchesPerSec2", Units.metersToInches(initialConstraints.maxAcceleration));

        this.setpointHeightInches = new LoggedTunableNumber("Climber/SetpointInches", Units.metersToInches(ClimberConstants.START));

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        // Set tuning mode to true
        kP.setTuningMode(true);
        kI.setTuningMode(true);
        kD.setTuningMode(true);
        kS.setTuningMode(true);
        kG.setTuningMode(true);
        kV.setTuningMode(true);
        kA.setTuningMode(true);
        maxVelocityInchesPerSec.setTuningMode(true);
        maxAccelerationInchesPerSec2.setTuningMode(true);
        setpointHeightInches.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Update PID only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> climber.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> climber.setFeedforward(new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get())),
            kS, kG, kV, kA);

        // Update trapezoid profile only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> climber.setProfile(new TrapezoidProfile(new Constraints(Units.inchesToMeters(maxVelocityInchesPerSec.get()), Units.inchesToMeters(maxAccelerationInchesPerSec2.get())))),
            maxVelocityInchesPerSec, maxAccelerationInchesPerSec2);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> climber.setHeight(Units.inchesToMeters(setpointHeightInches.get())),
            setpointHeightInches);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}