package org.frogforce503.robot.subsystems.launcher.flywheels.io;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot.subsystems.launcher.flywheels.FlywheelsConstants;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;
import lombok.Getter;

public class FlywheelsIOSpark implements FlywheelsIO {
    // Hardware
    @Getter private final SparkFlex leader;
    private final RelativeEncoder leaderEncoder;

    @Getter private final SparkFlex follower;
    private final RelativeEncoder followerEncoder;

    // Control
    private final SparkClosedLoopController leaderController;

    // Config
    private SparkFlexConfig leaderConfig = new SparkFlexConfig();
    private SparkFlexConfig followerConfig = new SparkFlexConfig();

    // Filters
    private final Debouncer leaderConnectedDebouncer = new Debouncer(.5);
    private final Debouncer followerConnectedDebouncer = new Debouncer(.5);
    
    public FlywheelsIOSpark() {
        // Initialize motor
        leader = new SparkFlex(FlywheelsConstants.leaderId, MotorType.kBrushless);
        leaderEncoder = leader.getEncoder();
        leaderController = leader.getClosedLoopController();

        follower = new SparkFlex(FlywheelsConstants.followerId, MotorType.kBrushless);
        followerEncoder = follower.getEncoder();

        // Configure motor
        leaderConfig.inverted(FlywheelsConstants.leaderInverted);
        leaderConfig.idleMode(IdleMode.kBrake);
        leaderConfig.smartCurrentLimit(FlywheelsConstants.statorCurrentLimit);
        leaderConfig.voltageCompensation(12.0);

        leaderConfig
            .encoder
                .positionConversionFactor((1 / FlywheelsConstants.motorMechanismRatio) * (2 * Math.PI)) // convert rotations to radians
                .velocityConversionFactor((1 / FlywheelsConstants.motorMechanismRatio) * (2 * Math.PI) / 60); // convert RPM to rad/sec
                
        SparkUtil.optimizeRelativeEncoderFilter(leaderConfig);

        leaderConfig
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(FlywheelsConstants.kPID.kP(), FlywheelsConstants.kPID.kI(), FlywheelsConstants.kPID.kD());

        SparkUtil.optimizeSignals(leaderConfig, false, false);

        followerConfig.apply(leaderConfig);
        followerConfig.follow(FlywheelsConstants.leaderId, FlywheelsConstants.followerInverted);

        leader.clearFaults();
        follower.clearFaults();

        // Apply config
        SparkUtil.configure(leader, leaderConfig, true);
        SparkUtil.configure(follower, followerConfig, true);
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        inputs.leaderConnected = leaderConnectedDebouncer.calculate(leader.getLastError() == REVLibError.kOk);
        inputs.leaderVelocityRadPerSec = leaderEncoder.getVelocity();
        inputs.leaderAppliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
        inputs.leaderStatorCurrentAmps = leader.getOutputCurrent();
        inputs.leaderTempCelsius = leader.getMotorTemperature();

        inputs.followerConnected = followerConnectedDebouncer.calculate(follower.getLastError() == REVLibError.kOk);
        inputs.followerVelocityRadPerSec = followerEncoder.getVelocity();
        inputs.followerAppliedVolts = follower.getAppliedOutput() * follower.getBusVoltage();
        inputs.followerStatorCurrentAmps = follower.getOutputCurrent();
        inputs.followerTempCelsius = follower.getMotorTemperature();
    }

    @Override
    public void runOpenLoop(double output) {
        leader.set(output);
    }

    @Override
    public void runVolts(double volts) {
        leader.setVoltage(volts);
    }

    @Override
    public void runVelocity(double velocityRadPerSec, double feedforward) {
        leaderController.setSetpoint(velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        leaderConfig.closedLoop.pid(kP, kI, kD);
        followerConfig.closedLoop.pid(kP, kI, kD);

        SparkUtil.configure(leader, leaderConfig, false);
        SparkUtil.configure(follower, followerConfig, false);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        leaderConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        followerConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);

        SparkUtil.configure(leader, leaderConfig, false);
        SparkUtil.configure(follower, followerConfig, false);
    }
}