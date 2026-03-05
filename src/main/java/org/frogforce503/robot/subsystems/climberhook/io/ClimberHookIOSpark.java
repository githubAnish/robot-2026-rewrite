package org.frogforce503.robot.subsystems.climberhook.io;

import java.time.Duration;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot.subsystems.climberhook.ClimberHookConstants;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import lombok.Getter;

public class ClimberHookIOSpark implements ClimberHookIO {
    // Hardware
    @Getter private final SparkMax motor;
    private final RelativeEncoder encoder;

    @Getter private final DigitalInput limitSwitch;

    // Control
    private final SparkClosedLoopController controller;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();

    // Filters
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    private final DigitalGlitchFilter limitSwitchFilter = new DigitalGlitchFilter();

    public ClimberHookIOSpark() {
        // Initialize motor
        motor = new SparkMax(ClimberHookConstants.id, MotorType.kBrushless);
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        // Initialize limit switch
        limitSwitch = new DigitalInput(ClimberHookConstants.limitSwitchId);
        limitSwitchFilter.setPeriodNanoSeconds(Duration.ofMillis(100).toNanos());
        limitSwitchFilter.add(limitSwitch);

        // Configure motor
        config.inverted(ClimberHookConstants.inverted);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ClimberHookConstants.statorCurrentLimit);
        config.voltageCompensation(12.0);

        config
            .encoder
                .positionConversionFactor((1 / ClimberHookConstants.mechanismRatio) * (ClimberHookConstants.sprocketPitchDiameter * Math.PI)) // convert rotations to meters
                .velocityConversionFactor((1 / ClimberHookConstants.mechanismRatio) * (ClimberHookConstants.sprocketPitchDiameter * Math.PI) / 60) // convert RPM to meters/sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ClimberHookConstants.kPID.kP(), ClimberHookConstants.kPID.kI(), ClimberHookConstants.kPID.kD());

        SparkUtil.optimizeSignals(config, false, false);

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);

        resetEncoder();
    }

    @Override
    public void updateInputs(ClimberHookIOInputs inputs) {
        inputs.motorConnected = connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk);
        inputs.positionMeters = encoder.getPosition();
        inputs.velocityMetersPerSec = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.statorCurrentAmps = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();
        inputs.limitSwitchPressed = !limitSwitch.get();
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
    public void runPosition(double positionMeters, double feedforward) {
        controller.setSetpoint(positionMeters, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
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

    @Override
    public void resetEncoder() {
        encoder.setPosition(0.0);
    }
}