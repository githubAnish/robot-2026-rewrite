package org.frogforce503.robot.subsystems.superstructure.intakepivot.io;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import lombok.Getter;

public class IntakePivotIOSpark implements IntakePivotIO {
    // Hardware
    @Getter private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;

    // Control
    private final SparkClosedLoopController controller;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();

    // Filters
    private final Debouncer connectedDebouncer = new Debouncer(.5);

    public IntakePivotIOSpark() {
        // Initialize motor
        motor = new SparkMax(IntakePivotConstants.id, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();

        // Configure motor
        config.inverted(IntakePivotConstants.inverted);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(IntakePivotConstants.statorCurrentLimit);
        config.voltageCompensation(12.0);

        config
            .absoluteEncoder
                .zeroOffset(IntakePivotConstants.zeroOffset)
                .positionConversionFactor(2 * Math.PI) // convert rotations to radians, TODO assume absolute encoder on main rotating shaft of intake pivot
                .velocityConversionFactor(2 * Math.PI / 60) // convert RPM to rad/sec, TODO assume absolute encoder on main rotating shaft of intake pivot
                .zeroCentered(true)
                .averageDepth(2)
                .setSparkMaxDataPortConfig();

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(IntakePivotConstants.kPID.kP(), IntakePivotConstants.kPID.kI(), IntakePivotConstants.kPID.kD());

        config
            .softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(IntakePivotConstants.maxAngle)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(IntakePivotConstants.minAngle);

        SparkUtil.optimizeSignals(config, true, false);

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        inputs.motorConnected = connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk);
        inputs.positionRad = encoder.getPosition();
        inputs.velocityRadPerSec = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.statorCurrentAmps = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void runOpenLoop(double output) {
        motor.set(output);
    }

    @Override
    public void runVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {
        controller.setSetpoint(positionRad, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.closedLoop.pid(kP, kI, kD);
        SparkUtil.configure(motor, config, false);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        SparkUtil.configure(motor, config, false);
    }
}