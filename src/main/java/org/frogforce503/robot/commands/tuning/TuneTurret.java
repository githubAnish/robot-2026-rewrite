package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneTurret extends Command {
    private final Turret turret;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber maxVelocityDegPerSec;
    private final LoggedTunableNumber maxAccelerationDegPerSec2;

    private final LoggedTunableNumber setpointAngleDeg;

    public TuneTurret(Turret turret) {
        this.turret = turret;

        // Get initial values from config
        final PIDConfig initialPID = TurretConstants.kPID;
        final FFConfig initialFF = TurretConstants.kFF;
        final Constraints initialConstraints = TurretConstants.kConstraints;

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("Turret/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("Turret/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("Turret/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("Turret/kS", initialFF.kS());
        this.kV = new LoggedTunableNumber("Turret/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("Turret/kA", initialFF.kA());

        this.maxVelocityDegPerSec = new LoggedTunableNumber("Turret/MaxVelocityDegPerSec", Units.radiansToDegrees(initialConstraints.maxVelocity));
        this.maxAccelerationDegPerSec2 = new LoggedTunableNumber("Turret/MaxAccelerationDegPerSec2", Units.radiansToDegrees(initialConstraints.maxAcceleration));

        this.setpointAngleDeg = new LoggedTunableNumber("Turret/SetpointDeg", Units.radiansToDegrees(TurretConstants.START));

        addRequirements(turret);
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
        this.maxVelocityDegPerSec.setTuningMode(true);
        this.maxAccelerationDegPerSec2.setTuningMode(true);
        this.setpointAngleDeg.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Update PID only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> turret.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> turret.setFeedforward(new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get())),
            kS, kV, kA);

        // Update trapezoid profile only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> turret.setProfile(new TrapezoidProfile(new Constraints(Units.degreesToRadians(maxVelocityDegPerSec.get()), Units.degreesToRadians(maxAccelerationDegPerSec2.get())))),
            maxVelocityDegPerSec, maxAccelerationDegPerSec2);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> turret.setAngle(Units.degreesToRadians(setpointAngleDeg.get())),
            setpointAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}