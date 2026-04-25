package org.frogforce503.robot.subsystems.launcher.hood.io;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot.subsystems.launcher.hood.HoodConstants;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import lombok.Getter;

public class HoodIOSpark implements HoodIO {
    // Hardware
    @Getter private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;

    // Control
    private final SparkClosedLoopController controller;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();

    // Filters
    private final Debouncer connectedDebouncer = new Debouncer(.5);

    public HoodIOSpark() {
        // Initialize motor
        motor = new SparkMax(HoodConstants.motorId, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();

        // Configure motor
        config.inverted(HoodConstants.motorInverted);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(HoodConstants.statorCurrentLimit);
        config.voltageCompensation(12.0);

        config
            .absoluteEncoder
                .inverted(HoodConstants.absoluteEncoderInverted)
                .zeroCentered(true)
                .zeroOffset(HoodConstants.absoluteEncoderZeroOffset)
                .positionConversionFactor((1 / HoodConstants.absoluteEncoderMechanismRatio) * 2 * Math.PI) // convert rotations to radians
                .velocityConversionFactor((1 / HoodConstants.absoluteEncoderMechanismRatio) * 2 * Math.PI / 60) // convert RPM to rad/sec
                .averageDepth(2)
                .setSparkMaxDataPortConfig();

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(HoodConstants.kPID.kP(), HoodConstants.kPID.kI(), HoodConstants.kPID.kD());

        SparkUtil.optimizeSignals(config, true, false);

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
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