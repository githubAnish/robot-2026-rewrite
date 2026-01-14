package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.IntakePivotConfig;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneIntakePivot extends Command {
    private final IntakePivot intakePivot;

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

    public TuneIntakePivot(IntakePivot intakePivot) {
        this.intakePivot = intakePivot;

        // Get initial values from config
        final IntakePivotConfig intakePivotConfig = Robot.bot.getIntakePivotConfig();

        final PIDConfig initialPID = intakePivotConfig.kPID();
        final FFConfig initialFF = intakePivotConfig.kFF();
        final Constraints initialConstraints = intakePivotConfig.kConstraints();

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("IntakePivot/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("IntakePivot/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("IntakePivot/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("IntakePivot/kS", initialFF.kS());
        this.kG = new LoggedTunableNumber("IntakePivot/kG", initialFF.kG());
        this.kV = new LoggedTunableNumber("IntakePivot/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("IntakePivot/kA", initialFF.kA());

        this.maxVelocityDegPerSec = new LoggedTunableNumber("IntakePivot/MaxVelocityDegPerSec", Units.radiansToDegrees(initialConstraints.maxVelocity));
        this.maxAccelerationDegPerSec2 = new LoggedTunableNumber("IntakePivot/MaxAccelerationDegPerSec2", Units.radiansToDegrees(initialConstraints.maxAcceleration));

        this.setpointAngleDeg = new LoggedTunableNumber("IntakePivot/SetpointDeg", Units.radiansToDegrees(IntakePivotConstants.START));

        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
        // Set tuning mode to true
        this.kP.setTuningMode(true);
        this.kI.setTuningMode(true);
        this.kD.setTuningMode(true);
        this.kS.setTuningMode(true);
        this.kG.setTuningMode(true);
        this.kV.setTuningMode(true);
        this.kA.setTuningMode(true);
        this.maxVelocityDegPerSec.setTuningMode(true);
        this.maxAccelerationDegPerSec2.setTuningMode(true);
        this.setpointAngleDeg.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Update PID only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakePivot.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakePivot.setFeedforward(new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get())),
            kS, kG, kV, kA);

        // Update trapezoid profile only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakePivot.setProfile(new TrapezoidProfile(new Constraints(Units.degreesToRadians(maxVelocityDegPerSec.get()), Units.degreesToRadians(maxAccelerationDegPerSec2.get())))),
            maxVelocityDegPerSec, maxAccelerationDegPerSec2);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakePivot.setAngle(Units.degreesToRadians(setpointAngleDeg.get())),
            setpointAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.stop();
    }
}