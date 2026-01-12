package org.frogforce503.robot.subsystems.leds.io;

import org.frogforce503.robot.Robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class LedsIOCANdle implements LedsIO {
    private final CANdle leds;

    private final StatusSignal<Voltage> supplyVolts;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temp;

    public LedsIOCANdle() {
        // Initialize CANdle
        leds = new CANdle(Robot.bot.getLedsConfig().candleID());

        // Apply config
        CANdleConfiguration config = new CANdleConfiguration();

        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = 0.75;

        leds.getConfigurator().apply(config);

        // Initialize signals
        supplyVolts = leds.getSupplyVoltage();
        statorCurrent = leds.getOutputCurrent();
        temp = leds.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            4, // Update 4 times a second
            supplyVolts,
            statorCurrent,
            temp);

        leds.optimizeBusUtilization();

        // Clear all animations
        for (int i = 0; i < 8; i++) {
            leds.setControl(new EmptyAnimation(i));
        }
    }

    @Override
    public void updateInputs(LedsIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            supplyVolts,
            statorCurrent,
            temp);

        inputs.stripConnected = BaseStatusSignal.isAllGood(supplyVolts, statorCurrent, temp);
        inputs.patternName = leds.getAppliedControl().getName();
        inputs.supplyVolts = supplyVolts.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    @Override
    public void runPattern(ControlRequest pattern) {
        leds.setControl(pattern);
    }
}