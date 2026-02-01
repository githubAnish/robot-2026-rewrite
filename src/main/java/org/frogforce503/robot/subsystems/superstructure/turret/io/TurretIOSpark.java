package org.frogforce503.robot.subsystems.superstructure.turret.io;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
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

public class TurretIOSpark implements TurretIO {
    // Hardware
    @Getter private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkAbsoluteEncoder absoluteEncoder;

    // Control
    private final SparkClosedLoopController controller;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();

    // Filters
    private final Debouncer connectedDebouncer = new Debouncer(.5);

    public TurretIOSpark() {
        // Initialize motor
        motor = new SparkMax(TurretConstants.id, MotorType.kBrushless);
        encoder = motor.getEncoder();
        absoluteEncoder = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();

        // Configure motor
        config.inverted(TurretConstants.inverted);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(TurretConstants.statorCurrentLimit);
        config.voltageCompensation(12.0);

        config
            .encoder
                .positionConversionFactor((1 / TurretConstants.mechanismRatio) * (2 * Math.PI)) // convert rotations to radians
                .velocityConversionFactor((1 / TurretConstants.mechanismRatio) * (2 * Math.PI) / 60) // convert RPM to rad/sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        config
            .absoluteEncoder
                .zeroOffset(TurretConstants.zeroOffset)
                .positionConversionFactor(2 * Math.PI) // convert rotations to radians, TODO assume absolute encoder on main rotating shaft of turret
                .velocityConversionFactor(2 * Math.PI / 60) // convert RPM to rad/sec, TODO assume absolute encoder on main rotating shaft of turret
                .zeroCentered(true)
                .averageDepth(2)
                .setSparkMaxDataPortConfig();

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(TurretConstants.kPID.kP(), TurretConstants.kPID.kI(), TurretConstants.kPID.kD());

        config
            .softLimit // TODO Soft limits especially important for a turret, as wires can snap due to over-rotation
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(TurretConstants.maxAngle)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(TurretConstants.minAngle);

        SparkUtil.optimizeSignals(config, true, false);

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);

        setRelativePosition(absoluteEncoder.getPosition());
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.motorConnected = connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk);
        inputs.positionRad = encoder.getPosition();
        inputs.absolutePositionRad = absoluteEncoder.getPosition();
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

    @Override
    public void setRelativePosition(double positionRad) {
        encoder.setPosition(positionRad);
    }
}