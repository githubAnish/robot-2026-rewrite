package org.frogforce503.lib.motorcontrol;

import org.frogforce503.lib.util.ErrorUtil;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;

/** Helper class for Spark IO implementations */
public final class SparkUtil {
    public static final int kDefaultCurrentLimit = 80;
    public static final int kThroughBoreEncoderCounts = 8192;

    private SparkUtil() {}

    public static ClosedLoopSlot getClosedLoopSlot(int slot) {
        assert (0 <= slot && slot <= 3) : "Invalid slot ID: " + slot + ErrorUtil.attachJavaClassName(SparkUtil.class);
        return ClosedLoopSlot.values()[slot];
    }

    public static <S extends SparkBase, C extends SparkBaseConfig> void configure(S motor, C config, boolean burnFlash) {
        motor.configure(
            config,
            burnFlash ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters,
            burnFlash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    }

    /** Optimizes motor signals to limit unnecessary data over CAN. */
    public static <C extends SparkBaseConfig> void optimizeSignals(C config, boolean hasAbsoluteEncoder, boolean hasExternalOrAlternateEncoder) {
        config
            .signals
                .absoluteEncoderPositionAlwaysOn(hasAbsoluteEncoder)
                .absoluteEncoderVelocityAlwaysOn(hasAbsoluteEncoder)
                .analogPositionAlwaysOn(false) // PLEASE uncomment if there's analog sensors attached to motor controller
                .analogVelocityAlwaysOn(false) // PLEASE uncomment if there's analog sensors attached to motor controller
                .analogVoltageAlwaysOn(false) // PLEASE uncomment if there's analog sensors attached to motor controller
                .externalOrAltEncoderPositionAlwaysOn(hasExternalOrAlternateEncoder)
                .externalOrAltEncoderVelocityAlwaysOn(hasExternalOrAlternateEncoder)
                .faultsAlwaysOn(true)
                .faultsPeriodMs(1000) // Updates once a second, faults don't need high update rate
                .iAccumulationAlwaysOn(true)
                .limitsPeriodMs(250) // Updates 4 times a second, PLEASE uncomment if there are limit switches attached to motor controller
                .motorTemperaturePeriodMs(1000) // Updates once a second, temperature doesn't need high update rate
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .warningsAlwaysOn(true)
                .warningsPeriodMs(1000); // Updates once a second, warnings don't need high update rate
    }
}