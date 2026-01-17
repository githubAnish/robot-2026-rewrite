package org.frogforce503.robot.subsystems.superstructure.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.subsystems.superstructure.turret.io.TurretIO;
import org.frogforce503.robot.subsystems.superstructure.turret.io.TurretIOInputsAutoLogged;

public class Turret extends FFSubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward;

    // Control
    private double targetAngleRad = TurretConstants.START;

    private boolean shouldRunProfile = true;
    @Setter private SlewRateLimiter profile;
    @Getter private double setpoint = 0.0;
    private boolean atGoal = false;

    public Turret(TurretIO io) {
        this.io = io;

        feedforward = Robot.bot.getTurretConfig().kFF().getSimpleMotorFF();
        profile = new SlewRateLimiter(Robot.bot.getTurretConfig().kRateLimit());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            double goalAngleRad = MathUtil.clamp(targetAngleRad, TurretConstants.minAngle, TurretConstants.maxAngle);

            double previousSetpoint = setpoint;

            setpoint = profile.calculate(goalAngleRad);
            atGoal = isAtAngle(goalAngleRad, TurretConstants.kTolerance);

            double velocity = (setpoint - previousSetpoint) / Constants.loopPeriodSecs;
            io.runPosition(setpoint, feedforward.calculate(velocity));

            // Log state
            Logger.recordOutput("Turret/Profile/SetpointPositionRad", setpoint);
            Logger.recordOutput("Turret/Profile/SetpointVelocityRadPerSec", velocity);
            Logger.recordOutput("Turret/Profile/GoalPositionRad", goalAngleRad);
            Logger.recordOutput("Turret/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = getAngleRad();
            profile.reset(setpoint);
      
            // Clear logs
            Logger.recordOutput("Turret/Profile/SetpointPositionRad", setpoint);
            Logger.recordOutput("Turret/Profile/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Turret/Profile/GoalPositionRad", 0.0);
            Logger.recordOutput("Turret/AtGoal", true);
        }

        Logger.recordOutput("Turret/CurrentPositionRad", getAngleRad());

        // Record cycle time
        LoggedTracer.record("Turret");
    }

    public double getAngleRad() {
        return inputs.data.positionRad();
    }

    // Actions
    public void seedRelativePosition() {
        io.setRelativePosition(inputs.data.absolutePositionRad());
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