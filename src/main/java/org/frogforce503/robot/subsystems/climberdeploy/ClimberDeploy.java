package org.frogforce503.robot.subsystems.climberdeploy;

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
import org.frogforce503.robot.subsystems.climberdeploy.io.ClimberDeployIO;
import org.frogforce503.robot.subsystems.climberdeploy.io.ClimberDeployIOInputsAutoLogged;

public class ClimberDeploy extends FFSubsystemBase {
    private final ClimberDeployIO io;
    private final ClimberDeployIOInputsAutoLogged inputs = new ClimberDeployIOInputsAutoLogged();

    // Constants
    @Setter private ArmFeedforward feedforward;

    // Control
    private double targetAngleRad = ClimberDeployConstants.START;

    private boolean shouldRunProfile = true;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public ClimberDeploy(ClimberDeployIO io) {
        this.io = io;

        feedforward = ClimberDeployConstants.kFF.getArmFF();
        profile = new TrapezoidProfile(ClimberDeployConstants.kConstraints);
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("ClimberDeploy", inputs);

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    MathUtil.clamp(targetAngleRad, ClimberDeployConstants.minAngle, ClimberDeployConstants.maxAngle),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
            atGoal = isAtAngle(goalState.position, ClimberDeployConstants.kTolerance);

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
            io.runPosition(setpoint.position, feedforward.calculate(setpoint.position, setpoint.velocity, accel));

            // Log state
            Logger.recordOutput("ClimberDeploy/Profile/SetpointPositionRad", setpoint.position);
            Logger.recordOutput("ClimberDeploy/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
            Logger.recordOutput("ClimberDeploy/Profile/GoalPositionRad", goalState.position);
            Logger.recordOutput("ClimberDeploy/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getAngleRad(), 0.0);
      
            // Clear logs
            Logger.recordOutput("ClimberDeploy/Profile/SetpointPositionRad", 0.0);
            Logger.recordOutput("ClimberDeploy/Profile/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("ClimberDeploy/Profile/GoalPositionRad", 0.0);
            Logger.recordOutput("ClimberDeploy/AtGoal", true);
        }

        Logger.recordOutput("ClimberDeploy/CurrentPositionRad", getAngleRad());

        // Record cycle time
        LoggedTracer.record("ClimberDeploy");
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