package org.frogforce503.robot.subsystems.climberhook;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.climberhook.io.ClimberHookIOInputsAutoLogged;
import org.frogforce503.robot.subsystems.climberhook.io.ClimberHookIO;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

public class ClimberHook extends FFSubsystemBase {
    private final ClimberHookIO io;
    private final ClimberHookIOInputsAutoLogged inputs = new ClimberHookIOInputsAutoLogged();

    // Constants
    @Setter private ElevatorFeedforward feedforward;
    
    // Control
    private double targetHeightMeters = ClimberHookConstants.START;
    private double lastHeightMeters = 0.0;

    private boolean shouldRunProfile = true;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public ClimberHook(ClimberHookIO io) {
        this.io = io;
        
        feedforward = ClimberHookConstants.kFF.getElevatorFF();
        profile = new TrapezoidProfile(ClimberHookConstants.kConstraints);
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("ClimberHook", inputs);

        // Reset encoder if limit switch pressed & climber is going down
        if (inputs.limitSwitchPressed && getHeightMeters() < lastHeightMeters) {
            double heightAtLimitSwitch = ClimberHookConstants.minHeight; // assume limit switch at bottom

            setRelativePosition(heightAtLimitSwitch);
            setpoint = new State(heightAtLimitSwitch, 0.0);
        }

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    MathUtil.clamp(targetHeightMeters, ClimberHookConstants.minHeight, ClimberHookConstants.maxHeight),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
            atGoal = isAtHeight(goalState.position, ClimberHookConstants.tolerance);

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
            io.runPosition(setpoint.position, feedforward.calculate(setpoint.velocity, accel));

            /// Log state
            Logger.recordOutput("ClimberHook/Profile/SetpointPositionMeters", setpoint.position);
            Logger.recordOutput("ClimberHook/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
            Logger.recordOutput("ClimberHook/Profile/GoalPositionMeters", goalState.position);
            Logger.recordOutput("ClimberHook/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getHeightMeters(), 0.0);
      
            // Clear logs
            Logger.recordOutput("ClimberHook/Profile/SetpointPositionMeters", 0.0);
            Logger.recordOutput("ClimberHook/Profile/SetpointVelocityMetersPerSec", 0.0);
            Logger.recordOutput("ClimberHook/Profile/GoalPositionMeters", 0.0);
            Logger.recordOutput("ClimberHook/AtGoal", true);
        }

        Logger.recordOutput("ClimberHook/CurrentPositionMeters", getHeightMeters());
        lastHeightMeters = getHeightMeters();

        // Record cycle time
        LoggedTracer.record("ClimberHook");
    }

    public double getHeightMeters() {
        return inputs.positionMeters;
    }

    public double getVelocityMetersPerSec() {
        return inputs.velocityMetersPerSec;
    }

    // Actions
    public void setRelativePosition(double positionMeters) {
        io.setRelativePosition(positionMeters);
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
        shouldRunProfile = false;

        // Prevent downward motion into limit switch
        if (inputs.limitSwitchPressed && volts < 0) {
            volts = 0;
        }

        io.runVolts(volts);
    }

    public void setHeight(double heightMeters) {
        shouldRunProfile = true;
        targetHeightMeters = heightMeters;
    }

    public boolean isAtHeight(double heightMeters, double tolerance) {
        return MathUtil.isNear(heightMeters, getHeightMeters(), tolerance);
    }
}