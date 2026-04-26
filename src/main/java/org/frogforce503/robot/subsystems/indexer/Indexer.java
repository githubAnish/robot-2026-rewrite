package org.frogforce503.robot.subsystems.indexer;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.subsystems.indexer.io.IndexerIO;
import org.frogforce503.robot.subsystems.indexer.io.IndexerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Setter;

public class Indexer extends FFSubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward;

    // Control
    private double targetVelocityRadPerSec = IndexerConstants.START;

    private boolean shouldRunVelocity = false;
    
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
            boolean atGoal = isAtVelocity(targetVelocityRadPerSec, IndexerConstants.tolerance);
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
        shouldRunVelocity = false;
        io.runVolts(volts);
    }

    public void setVelocity(double velocityRadPerSec) {
        shouldRunVelocity = true;
        targetVelocityRadPerSec = velocityRadPerSec;
    }

    public boolean isAtVelocity(double velocityRadPerSec, double tolerance) {
        return MathUtil.isNear(velocityRadPerSec, getVelocityRadPerSec(), tolerance);
    }
}