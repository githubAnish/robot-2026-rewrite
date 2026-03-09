package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.subsystems.climberdeploy.ClimberDeploy;
import org.frogforce503.robot.subsystems.climberdeploy.ClimberDeployConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneClimberDeploy extends Command {
    private final ClimberDeploy climberDeploy;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kG;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber maxVelocityDegPerSec;
    private final LoggedTunableNumber maxAccelerationDegPerSec2;

    private final LoggedTunableNumber setpointAngleDeg;

    public TuneClimberDeploy(ClimberDeploy climberDeploy) {
        this.climberDeploy = climberDeploy;

        // Get initial values from config
        final PIDConfig initialPID = ClimberDeployConstants.kPID;
        final FFConfig initialFF = ClimberDeployConstants.kFF;
        final Constraints initialConstraints = ClimberDeployConstants.kConstraints;

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("ClimberDeploy/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("ClimberDeploy/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("ClimberDeploy/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("ClimberDeploy/kS", initialFF.kS());
        this.kG = new LoggedTunableNumber("ClimberDeploy/kG", initialFF.kG());
        this.kV = new LoggedTunableNumber("ClimberDeploy/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("ClimberDeploy/kA", initialFF.kA());

        this.maxVelocityDegPerSec = new LoggedTunableNumber("ClimberDeploy/MaxVelocityDegPerSec", Units.radiansToDegrees(initialConstraints.maxVelocity));
        this.maxAccelerationDegPerSec2 = new LoggedTunableNumber("ClimberDeploy/MaxAccelerationDegPerSec2", Units.radiansToDegrees(initialConstraints.maxAcceleration));

        this.setpointAngleDeg = new LoggedTunableNumber("ClimberDeploy/SetpointDeg", Units.radiansToDegrees(ClimberDeployConstants.START));

        addRequirements(climberDeploy);
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
        maxVelocityDegPerSec.setTuningMode(true);
        maxAccelerationDegPerSec2.setTuningMode(true);
        setpointAngleDeg.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Update PID only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> climberDeploy.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> climberDeploy.setFeedforward(new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get())),
            kS, kG, kV, kA);

        // Update trapezoid profile only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> climberDeploy.setProfile(new TrapezoidProfile(new Constraints(Units.degreesToRadians(maxVelocityDegPerSec.get()), Units.degreesToRadians(maxAccelerationDegPerSec2.get())))),
            maxVelocityDegPerSec, maxAccelerationDegPerSec2);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> climberDeploy.setAngle(Units.degreesToRadians(setpointAngleDeg.get())),
            setpointAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climberDeploy.stop();
    }
}