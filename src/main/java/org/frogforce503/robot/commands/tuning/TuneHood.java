package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneHood extends Command {
    private final Hood hood;

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

    public TuneHood(Hood hood) {
        this.hood = hood;

        // Get initial values from config
        final PIDConfig initialPID = HoodConstants.kPID;
        final FFConfig initialFF = HoodConstants.kFF;
        final Constraints initialConstraints = HoodConstants.kConstraints;

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("Hood/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("Hood/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("Hood/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("Hood/kS", initialFF.kS());
        this.kG = new LoggedTunableNumber("Hood/kG", initialFF.kG());
        this.kV = new LoggedTunableNumber("Hood/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("Hood/kA", initialFF.kA());

        this.maxVelocityDegPerSec = new LoggedTunableNumber("Hood/MaxVelocityDegPerSec", Units.radiansToDegrees(initialConstraints.maxVelocity));
        this.maxAccelerationDegPerSec2 = new LoggedTunableNumber("Hood/MaxAccelerationDegPerSec2", Units.radiansToDegrees(initialConstraints.maxAcceleration));

        this.setpointAngleDeg = new LoggedTunableNumber("Hood/SetpointDeg", Units.radiansToDegrees(HoodConstants.START));

        addRequirements(hood);
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
            () -> hood.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> hood.setFeedforward(new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get())),
            kS, kG, kV, kA);

        // Update trapezoid profile only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> hood.setProfile(new TrapezoidProfile(new Constraints(Units.degreesToRadians(maxVelocityDegPerSec.get()), Units.degreesToRadians(maxAccelerationDegPerSec2.get())))),
            maxVelocityDegPerSec, maxAccelerationDegPerSec2);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> hood.setAngle(Units.degreesToRadians(setpointAngleDeg.get())),
            setpointAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        hood.stop();
    }
}