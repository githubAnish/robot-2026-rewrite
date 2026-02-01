package org.frogforce503.robot.subsystems.superstructure.intakepivot;

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
import org.frogforce503.robot.subsystems.superstructure.intakepivot.io.IntakePivotIO;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.io.IntakePivotIOInputsAutoLogged;

// Assume the intakepivot is just a 4-bar intake that stows inward & deploys out
public class IntakePivot extends FFSubsystemBase {
    private final IntakePivotIO io;
    private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

    // Constants
    @Setter private ArmFeedforward feedforward;

    // Control
    private double targetAngleRad = IntakePivotConstants.START;

    private boolean shouldRunProfile = true;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public IntakePivot(IntakePivotIO io) {
        this.io = io;

        feedforward = IntakePivotConstants.kFF.getArmFF();
        profile = new TrapezoidProfile(IntakePivotConstants.kConstraints);
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    MathUtil.clamp(targetAngleRad, IntakePivotConstants.minAngle, IntakePivotConstants.maxAngle),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
            atGoal = isAtAngle(goalState.position, IntakePivotConstants.kTolerance);

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
            io.runPosition(setpoint.position, feedforward.calculate(setpoint.position, setpoint.velocity, accel));

            // Log state
            Logger.recordOutput("IntakePivot/Profile/SetpointPositionRad", setpoint.position);
            Logger.recordOutput("IntakePivot/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
            Logger.recordOutput("IntakePivot/Profile/GoalPositionRad", goalState.position);
            Logger.recordOutput("IntakePivot/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getAngleRad(), 0.0);
      
            // Clear logs
            Logger.recordOutput("IntakePivot/Profile/SetpointPositionRad", 0.0);
            Logger.recordOutput("IntakePivot/Profile/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("IntakePivot/Profile/GoalPositionRad", 0.0);
            Logger.recordOutput("IntakePivot/AtGoal", true);
        }

        Logger.recordOutput("IntakePivot/CurrentPositionRad", getAngleRad());

        // Record cycle time
        LoggedTracer.record("IntakePivot");
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
        this.shouldRunProfile = true;
        this.targetAngleRad = angleRad;
    }

    public boolean isAtAngle(double angleRad, double tolerance) {
        return MathUtil.isNear(angleRad, getAngleRad(), tolerance);
    }
}