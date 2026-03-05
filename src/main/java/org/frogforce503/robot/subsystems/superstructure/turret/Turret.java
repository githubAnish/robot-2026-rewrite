package org.frogforce503.robot.subsystems.superstructure.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.robot.Constants;
import org.frogforce503.robot.subsystems.superstructure.turret.io.TurretIO;
import org.frogforce503.robot.subsystems.superstructure.turret.io.TurretIOInputsAutoLogged;

public class Turret extends FFSubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private final Supplier<Rotation2d> robotAngleSupplier;
    private final DoubleSupplier robotOmegaSupplier;

    // Constants
    @Setter private SimpleMotorFeedforward feedforward;

    // Control
    private double targetAngleRad = TurretConstants.START;
    private double targetVelocityRadPerSec = 0.0;
    private double lastTargetAngleRad = TurretConstants.START;

    private boolean shouldRunProfile = true;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public Turret(TurretIO io, Supplier<Rotation2d> robotAngleSupplier, DoubleSupplier robotOmegaSupplier) {
        this.io = io;

        this.robotAngleSupplier = robotAngleSupplier;
        this.robotOmegaSupplier = robotOmegaSupplier;

        feedforward = TurretConstants.kFF.getSimpleMotorFF();
        profile = new TrapezoidProfile(TurretConstants.kConstraints);
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            // Calculate best angle
            boolean hasBestAngle = false;
            double bestAngle = 0;

            for (int i = -2; i < 3; i++) {
                double potentialSetpoint = targetAngleRad + Math.PI * 2.0 * i;

                if (potentialSetpoint < TurretConstants.minAngle || potentialSetpoint > TurretConstants.maxAngle) {
                    continue;

                } else {
                    if (!hasBestAngle) {
                        bestAngle = potentialSetpoint;
                        hasBestAngle = true;
                    }
                    
                    if (Math.abs(lastTargetAngleRad - potentialSetpoint) < Math.abs(lastTargetAngleRad - bestAngle)) {
                        bestAngle = potentialSetpoint;
                    }
                }
            }

            lastTargetAngleRad = bestAngle;

            // Run profile
            var goalState =
                new State(
                    MathUtil.clamp(bestAngle, TurretConstants.minAngle, TurretConstants.maxAngle),
                    targetVelocityRadPerSec);

            double previousVelocity = setpoint.velocity;

            setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
            atGoal = isAtAngle(goalState.position, TurretConstants.kFixedTolerance);

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
            io.runPosition(setpoint.position, feedforward.calculate(setpoint.velocity, accel));

            // Log state
            Logger.recordOutput("Turret/Profile/SetpointPositionRad", setpoint.position);
            Logger.recordOutput("Turret/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
            Logger.recordOutput("Turret/Profile/GoalPositionRad", goalState.position);
            Logger.recordOutput("Turret/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getRobotRelativeAngleRad(), 0.0);
      
            // Clear logs
            Logger.recordOutput("Turret/Profile/SetpointPositionRad", 0.0);
            Logger.recordOutput("Turret/Profile/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Turret/Profile/GoalPositionRad", 0.0);
            Logger.recordOutput("Turret/AtGoal", true);
        }

        Logger.recordOutput("Turret/CurrentPositionRad", getRobotRelativeAngleRad());

        // Record cycle time
        LoggedTracer.record("Turret");
    }

    /** Gets the turret's robot-relative angle. */
    public double getRobotRelativeAngleRad() {
        return inputs.positionRad;
    }

    /** Gets the turret's field-relative angle. */
    public Rotation2d getFieldRelativeAngle() {
        return new Rotation2d(getRobotRelativeAngleRad()).plus(robotAngleSupplier.get());
    }

    // Actions
    public void seedRelativePosition() {
        boolean allDevicesConnected = inputs.motorConnected; // Checks if absolute encoder connected
        boolean inRange = MathUtils.inRange(getRobotRelativeAngleRad(), -Math.PI, Math.PI); // Ensures within absolute encoder range
        boolean goingSlow = Math.abs(inputs.velocityRadPerSec) < Units.degreesToRadians(2); // Velocity should be low for accurate seeding

        if (allDevicesConnected && inRange && goingSlow) {
            io.setRelativePosition(inputs.absolutePositionRad);
        }
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

    /** Sets the turret's robot-relative angle and robot-relative velocity. */
    public void setRobotRelativeAngle(double angleRad, double velocityRadPerSec) {
        this.shouldRunProfile = true;
        this.targetAngleRad = angleRad;
        this.targetVelocityRadPerSec = velocityRadPerSec;
    }

    /** Sets the turret's field-relative angle and field-relative velocity. */
    public void setFieldRelativeAngle(Rotation2d angle, double velocityRadPerSec) {
        double robotRelativeAngle = angle.minus(robotAngleSupplier.get()).getRadians();
        double robotRelativeVelocity = velocityRadPerSec - robotOmegaSupplier.getAsDouble();

        setRobotRelativeAngle(robotRelativeAngle, robotRelativeVelocity);
    }

    /** Checks if an angle is within tolerance of the turret's robot-relative angle. */
    public boolean isAtAngle(double angleRad, double tolerance) {
        return MathUtil.isNear(angleRad, getRobotRelativeAngleRad(), tolerance);
    }
}