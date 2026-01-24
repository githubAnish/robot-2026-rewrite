package org.frogforce503.robot.subsystems.climber;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.subsystems.climber.io.ClimberIO;
import org.frogforce503.robot.subsystems.climber.io.ClimberIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

// Climber pivots + is a chain
public class Climber extends FFSubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    // Constants
    @Setter private ElevatorFeedforward feedforward;
    
    // Control
    private double targetHeightMeters = ClimberConstants.START;
    private double lastHeightMeters = 0.0;

    private boolean shouldRunProfile = true;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public Climber(ClimberIO io) {
        this.io = io;
        
        feedforward = Robot.bot.getClimberConfig().kFF().getElevatorFF();
        profile = new TrapezoidProfile(Robot.bot.getClimberConfig().kConstraints());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        // Reset encoder if limit switch pressed & Climber is going down
        if (inputs.data.limitSwitchPressed() && getHeightMeters() < lastHeightMeters) {
            io.resetEncoder();
            setpoint = new State(0.0, 0.0);
        }

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    MathUtil.clamp(targetHeightMeters, ClimberConstants.minHeight, ClimberConstants.maxHeight),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
            atGoal = isAtHeight(goalState.position, ClimberConstants.kTolerance);

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
            io.runPosition(setpoint.position, feedforward.calculate(setpoint.velocity, accel));

            /// Log state
            Logger.recordOutput("Climber/Profile/SetpointPositionMeters", setpoint.position);
            Logger.recordOutput("Climber/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
            Logger.recordOutput("Climber/Profile/GoalPositionMeters", goalState.position);
            Logger.recordOutput("Climber/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getHeightMeters(), 0.0);
      
            // Clear logs
            Logger.recordOutput("Climber/Profile/SetpointPositionMeters", 0.0);
            Logger.recordOutput("Climber/Profile/SetpointVelocityMetersPerSec", 0.0);
            Logger.recordOutput("Climber/Profile/GoalPositionMeters", 0.0);
            Logger.recordOutput("Climber/AtGoal", true);
        }

        Logger.recordOutput("Climber/CurrentPositionMeters", getHeightMeters());
        lastHeightMeters = getHeightMeters();

        // Record cycle time
        LoggedTracer.record("Climber");
    }

    public double getHeightMeters() {
        return inputs.data.positionMeters();
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
        shouldRunProfile = false;

        // Prevent downward motion into the limit switch
        if (inputs.data.limitSwitchPressed() && volts < 0) {
            volts = 0;
        }

        io.runVolts(volts);
    }

    public void setHeight(double heightMeters) {
        this.shouldRunProfile = true;
        this.targetHeightMeters = heightMeters;
    }

    public boolean isAtHeight(double heightMeters, double tolerance) {
        return MathUtil.isNear(heightMeters, getHeightMeters(), tolerance);
    }
}