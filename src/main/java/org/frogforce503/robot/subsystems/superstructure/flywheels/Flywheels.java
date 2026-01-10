package org.frogforce503.robot.subsystems.superstructure.flywheels;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.Robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Setter;

public class Flywheels extends FFSubsystemBase {
    private final FlywheelsIO io;
    private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward;
    private final Debouncer algaeDebouncer = new Debouncer(0.5);

    // Control
    private double targetVelocityRadPerSec = FlywheelsConstants.START;

    private boolean shouldRunVelocity = false;
    private boolean atGoal = false;

    public Flywheels(FlywheelsIO io) {
        this.io = io;

        feedforward = Robot.bot.getFlywheelsConfig().kFF().getSimpleMotorFF();
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Flywheels", inputs);

        // Run velocity mode unless requested to stop
        if (shouldRunVelocity && RobotState.isEnabled()) {
            atGoal = isAtVelocity(targetVelocityRadPerSec, FlywheelsConstants.kRollerTolerance);
            io.runVelocity(targetVelocityRadPerSec, feedforward.calculate(targetVelocityRadPerSec));

            // Log state
            Logger.recordOutput("Flywheels/SetpointVelocityRadPerSec", targetVelocityRadPerSec);
            Logger.recordOutput("Flywheels/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetVelocityRadPerSec = 0.0;

            // Clear logs
            Logger.recordOutput("Flywheels/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Flywheels/AtGoal", true);
        }

        Logger.recordOutput("Flywheels/CurrentVelocityRadPerSec", getVelocityRadPerSec());

        // Record cycle time
        LoggedTracer.record("Flywheels");
    }

    public double getVelocityRadPerSec() {
        return inputs.data.velocityRadPerSec();
    }

    public boolean algaeCurrentThresholdForHoldMet() {
        return algaeDebouncer.calculate(
            inputs.data.statorCurrentAmps() > 15);
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