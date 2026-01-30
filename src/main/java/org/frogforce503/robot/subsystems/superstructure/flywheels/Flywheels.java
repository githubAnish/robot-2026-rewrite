package org.frogforce503.robot.subsystems.superstructure.flywheels;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.io.FlywheelsIO;
import org.frogforce503.robot.subsystems.superstructure.flywheels.io.FlywheelsIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

public class Flywheels extends FFSubsystemBase {
    private final FlywheelsIO io;
    private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward;

    // Control
    private double targetVelocityRadPerSec = FlywheelsConstants.START;

    private boolean shouldRunProfile = false;
    @Setter private SlewRateLimiter profile;
    @Getter private double setpoint = 0.0;
    private boolean atGoal = false;

    public Flywheels(FlywheelsIO io) {
        this.io = io;

        feedforward = FlywheelsConstants.kFF.getSimpleMotorFF();
        profile = new SlewRateLimiter(FlywheelsConstants.kRateLimit);
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Flywheels", inputs);

        // Run velocity mode unless requested to stop
        if (shouldRunProfile && RobotState.isEnabled()) {
            double previousVelocity = setpoint;

            setpoint = profile.calculate(targetVelocityRadPerSec);
            atGoal = isAtVelocity(targetVelocityRadPerSec, FlywheelsConstants.kTolerance);

            double accel = (setpoint - previousVelocity) / Constants.loopPeriodSecs;
            io.runVelocity(targetVelocityRadPerSec, feedforward.calculate(targetVelocityRadPerSec, accel));

            // Log state
            Logger.recordOutput("Flywheels/Profile/SetpointVelocityRadPerSec", setpoint);
            Logger.recordOutput("Flywheels/Profile/GoalVelocityRadPerSec", targetVelocityRadPerSec);
            Logger.recordOutput("Flywheels/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = 0.0;
            targetVelocityRadPerSec = 0.0;

            // Clear logs
            Logger.recordOutput("Flywheels/Profile/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Flywheels/Profile/GoalVelocityRadPerSec", 0.0);
            Logger.recordOutput("Flywheels/AtGoal", true);
        }

        Logger.recordOutput("Flywheels/CurrentVelocityRadPerSec", getVelocityRadPerSec());

        // Record cycle time
        LoggedTracer.record("Flywheels");
    }

    public double getVelocityRadPerSec() {
        return inputs.data.velocityRadPerSec();
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
        this.shouldRunProfile = false;
        io.runVolts(volts);
    }

    public void setVelocity(double velocityRadPerSec) {
        this.shouldRunProfile = true;
        this.targetVelocityRadPerSec = velocityRadPerSec;
    }

    public boolean isAtVelocity(double velocityRadPerSec, double tolerance) {
        return MathUtil.isNear(velocityRadPerSec, getVelocityRadPerSec(), tolerance);
    }
}