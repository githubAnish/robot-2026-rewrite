package org.frogforce503.robot.subsystems.superstructure.intakeroller;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.io.IntakeRollerIO;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.io.IntakeRollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Setter;

public class IntakeRoller extends FFSubsystemBase {
    private final IntakeRollerIO io;
    private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward;

    // Control
    private double targetVelocityRadPerSec = IntakeRollerConstants.START;

    private boolean shouldRunVelocity = false;
    private boolean atGoal = false;

    public IntakeRoller(IntakeRollerIO io) {
        this.io = io;

        feedforward = IntakeRollerConstants.kFF.getSimpleMotorFF();
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("IntakeRoller", inputs);

        // Run velocity mode unless requested to stop
        if (shouldRunVelocity && RobotState.isEnabled()) {
            atGoal = isAtVelocity(targetVelocityRadPerSec, IntakeRollerConstants.kTolerance);
            io.runVelocity(targetVelocityRadPerSec, feedforward.calculate(targetVelocityRadPerSec));

            // Log state
            Logger.recordOutput("IntakeRoller/SetpointVelocityRadPerSec", targetVelocityRadPerSec);
            Logger.recordOutput("IntakeRoller/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetVelocityRadPerSec = 0.0;

            // Clear logs
            Logger.recordOutput("IntakeRoller/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("IntakeRoller/AtGoal", true);
        }

        Logger.recordOutput("IntakeRoller/CurrentVelocityRadPerSec", getVelocityRadPerSec());

        // Record cycle time
        LoggedTracer.record("IntakeRoller");
    }

    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
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