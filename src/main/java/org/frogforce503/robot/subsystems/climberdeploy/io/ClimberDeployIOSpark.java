package org.frogforce503.robot.subsystems.climberdeploy.io;

import java.time.Duration;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot.subsystems.climberdeploy.ClimberDeployConstants;

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
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import lombok.Getter;

public class ClimberDeployIOSpark implements ClimberDeployIO {
    // Hardware
    @Getter private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;
    
    @Getter private final DigitalInput limitSwitch;

    // Control
    private final SparkClosedLoopController controller;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();

    // Filters
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    private final DigitalGlitchFilter limitSwitchFilter = new DigitalGlitchFilter();

    public ClimberDeployIOSpark() {
        // Initialize motor
        motor = new SparkMax(ClimberDeployConstants.id, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();

        // Initialize limit switch
        limitSwitch = new DigitalInput(ClimberDeployConstants.limitSwitchId);
        limitSwitchFilter.setPeriodNanoSeconds(Duration.ofMillis(100).toNanos());
        limitSwitchFilter.add(limitSwitch);

        // Configure motor
        config.inverted(ClimberDeployConstants.inverted);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ClimberDeployConstants.statorCurrentLimit);
        config.voltageCompensation(12.0);

        config
            .absoluteEncoder
                .inverted(ClimberDeployConstants.absoluteEncoderInverted)
                .zeroOffset(ClimberDeployConstants.zeroOffset)
                .positionConversionFactor((1 / ClimberDeployConstants.absoluteEncoderMechanismRatio) * 2 * Math.PI) // convert rotations to radians, TODO assume absolute encoder on main rotating shaft of intake pivot
                .velocityConversionFactor((1 / ClimberDeployConstants.absoluteEncoderMechanismRatio) * 2 * Math.PI / 60) // convert RPM to rad/sec, TODO assume absolute encoder on main rotating shaft of intake pivot
                .zeroCentered(true)
                .averageDepth(2)
                .setSparkMaxDataPortConfig();

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(ClimberDeployConstants.kPID.kP(), ClimberDeployConstants.kPID.kI(), ClimberDeployConstants.kPID.kD());

        SparkUtil.optimizeSignals(config, true, false);

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(ClimberDeployIOInputs inputs) {
        inputs.motorConnected = connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk);
        inputs.positionRad = encoder.getPosition();
        inputs.velocityRadPerSec = encoder.getVelocity();
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