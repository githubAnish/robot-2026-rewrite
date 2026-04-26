package org.frogforce503.robot.subsystems.feeder;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.subsystems.feeder.io.FeederIO;
import org.frogforce503.robot.subsystems.feeder.io.FeederIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Setter;

// Assume the feeder is just a bunch of rollers connected to each other
public class Feeder extends FFSubsystemBase {
    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward;

    // Control
    private double targetVelocityRadPerSec = FeederConstants.START;

    private boolean shouldRunVelocity = false;
    
    public Feeder(FeederIO io) {
        this.io = io;

        feedforward = FeederConstants.kFF.getSimpleMotorFF();
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        // Run velocity mode unless requested to stop
        if (shouldRunVelocity && RobotState.isEnabled()) {
            boolean atGoal = isAtVelocity(targetVelocityRadPerSec, FeederConstants.tolerance);
            io.runVelocity(targetVelocityRadPerSec, feedforward.calculate(targetVelocityRadPerSec));

            // Log state
            Logger.recordOutput("Feeder/SetpointVelocityRadPerSec", targetVelocityRadPerSec);
            Logger.recordOutput("Feeder/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetVelocityRadPerSec = 0.0;

            // Clear logs
            Logger.recordOutput("Feeder/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Feeder/AtGoal", true);
        }

        Logger.recordOutput("Feeder/CurrentVelocityRadPerSec", getVelocityRadPerSec());

        // Record cycle time
        LoggedTracer.record("Feeder");
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