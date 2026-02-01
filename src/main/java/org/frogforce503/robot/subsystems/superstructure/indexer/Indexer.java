package org.frogforce503.robot.subsystems.superstructure.indexer;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.subsystems.superstructure.indexer.io.IndexerIO;
import org.frogforce503.robot.subsystems.superstructure.indexer.io.IndexerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Setter;

// need velocity control, but need some sort of pulsing mechanism to get the balls into feeder correctly according to ri3d videos
// Assume the indexer is just a spindexer that rotates around
public class Indexer extends FFSubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward;

    // Control
    private double targetVelocityRadPerSec = IndexerConstants.START;

    private boolean shouldRunVelocity = false;
    private boolean atGoal = false;

    public Indexer(IndexerIO io) {
        this.io = io;

        feedforward = IndexerConstants.kFF.getSimpleMotorFF();
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        // Run velocity mode unless requested to stop
        if (shouldRunVelocity && RobotState.isEnabled()) {
            atGoal = isAtVelocity(targetVelocityRadPerSec, IndexerConstants.kTolerance);
            io.runVelocity(targetVelocityRadPerSec, feedforward.calculate(targetVelocityRadPerSec));

            // Log state
            Logger.recordOutput("Indexer/SetpointVelocityRadPerSec", targetVelocityRadPerSec);
            Logger.recordOutput("Indexer/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetVelocityRadPerSec = 0.0;

            // Clear logs
            Logger.recordOutput("Indexer/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Indexer/AtGoal", true);
        }

        Logger.recordOutput("Indexer/CurrentVelocityRadPerSec", getVelocityRadPerSec());

        // Record cycle time
        LoggedTracer.record("Indexer");
    }

    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }

    public boolean isCompressed() {
        return
            shouldRunVelocity &&
            Math.abs(targetVelocityRadPerSec) > 1e-3 &&
            getVelocityRadPerSec() < IndexerConstants.MIN_FREE_SPEED &&
            inputs.statorCurrentAmps > IndexerConstants.COMPRESSION_CURRENT;
    }

    // Actions
    public void setPID(double kP, double kI, double kD) {
        io.setPID(kP, kI, kD);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    @Override
    public void stop() {
        io.stop();
    }

    public void runVolts(double volts) {
        this.shouldRunVelocity = false;
        io.runVolts(volts);
    }

    public void setVelocity(double velocityRadPerSec) {
        this.shouldRunVelocity = true;
        this.targetVelocityRadPerSec = velocityRadPerSec;
    }

    public boolean isAtVelocity(double velocityRadPerSec, double tolerance) {
        return MathUtil.isNear(velocityRadPerSec, getVelocityRadPerSec(), tolerance);
    }
}