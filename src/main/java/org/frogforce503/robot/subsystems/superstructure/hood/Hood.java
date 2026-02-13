package org.frogforce503.robot.subsystems.superstructure.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.superstructure.hood.io.HoodIO;
import org.frogforce503.robot.subsystems.superstructure.hood.io.HoodIOInputsAutoLogged;

public class Hood extends FFSubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    // Constants
    @Setter private ArmFeedforward feedforward;

    // Control
    private double targetAngleRad = HoodConstants.START;
    private double targetVelocityRadPerSec = 0.0;

    private boolean shouldRunProfile = true;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public Hood(HoodIO io) {
        this.io = io;

        feedforward = HoodConstants.kFF.getArmFF();
        profile = new TrapezoidProfile(HoodConstants.kConstraints);
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    MathUtil.clamp(targetAngleRad, HoodConstants.minAngle, HoodConstants.maxAngle),
                    targetVelocityRadPerSec);

            double previousVelocity = setpoint.velocity;

            setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
            atGoal = isAtAngle(goalState.position, HoodConstants.kTolerance);

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
            io.runPosition(setpoint.position, feedforward.calculate(setpoint.position, setpoint.velocity, accel));

            // Log state
            Logger.recordOutput("Hood/Profile/SetpointPositionRad", setpoint.position);
            Logger.recordOutput("Hood/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
            Logger.recordOutput("Hood/Profile/GoalPositionRad", goalState.position);
            Logger.recordOutput("Hood/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getAngleRad(), 0.0);
      
            // Clear logs
            Logger.recordOutput("Hood/Profile/SetpointPositionRad", 0.0);
            Logger.recordOutput("Hood/Profile/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Hood/Profile/GoalPositionRad", 0.0);
            Logger.recordOutput("Hood/AtGoal", true);
        }

        Logger.recordOutput("Hood/CurrentPositionRad", getAngleRad());

        // Record cycle time
        LoggedTracer.record("Hood");
    }

    public double getAngleRad() {
        return inputs.positionRad;
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

    public void setAngle(double angleRad) {
        setAngle(angleRad, 0.0);
    }

    public void setAngle(double angleRad, double velocityRadPerSec) {
        this.shouldRunProfile = true;
        this.targetAngleRad = angleRad;
        this.targetVelocityRadPerSec = velocityRadPerSec;
    }

    public boolean isAtAngle(double angleRad, double tolerance) {
        return MathUtil.isNear(angleRad, getAngleRad(), tolerance);
    }
}