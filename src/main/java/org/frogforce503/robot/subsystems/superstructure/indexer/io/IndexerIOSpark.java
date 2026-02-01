package org.frogforce503.robot.subsystems.superstructure.indexer.io;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot.subsystems.superstructure.indexer.IndexerConstants;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import lombok.Getter;

public class IndexerIOSpark implements IndexerIO {
    // Hardware
    @Getter private final SparkBase motor;
    private final RelativeEncoder encoder;

    // Control
    private final SparkClosedLoopController controller;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();

    // Filters
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    
    public IndexerIOSpark() {
        // Initialize motor
        motor = new SparkMax(IndexerConstants.id, MotorType.kBrushless);
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        // Configure motor
        config.inverted(IndexerConstants.inverted);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(IndexerConstants.statorCurrentLimit);
        config.voltageCompensation(12.0);

        config
            .encoder
                .positionConversionFactor((1 / IndexerConstants.mechanismRatio) * (2 * Math.PI)) // convert rotations to radians
                .velocityConversionFactor((1 / IndexerConstants.mechanismRatio) * (2 * Math.PI) / 60) // convert RPM to rad/sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(IndexerConstants.kPID.kP(), IndexerConstants.kPID.kI(), IndexerConstants.kPID.kD());

        SparkUtil.optimizeSignals(config, false, false);

        motor.clearFaults();

        // Apply config
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.motorConnected = connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk);
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
    public void runVelocity(double velocityRadPerSec, double feedforward) {
        controller.setSetpoint(velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
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