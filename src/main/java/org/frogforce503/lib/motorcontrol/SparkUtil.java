package org.frogforce503.lib.motorcontrol;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Helper class for Spark IO implementations. */
public class SparkUtil {
    private SparkUtil() {}

    public static <S extends SparkBase, C extends SparkBaseConfig> void configure(S motor, C config, boolean burnFlash) {
        motor.configure(
            config,
            burnFlash ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters,
            burnFlash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    }

    /** <p> Changes the filtering for an internal encoder for a Spark Max or Spark Flex (depends on {@code isSparkFlex}). </p>
     *  <p> See https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567. </p>
     */
    public static <C extends SparkBaseConfig> void optimizeRelativeEncoderFilter(C config) {
        if (config instanceof SparkFlexConfig) { // Spark Flex uses quadrature internal encoder
            config
                .encoder
                    .quadratureMeasurementPeriod(5)
                    .quadratureAverageDepth(2);
        } else { // Spark Max uses hall-effect internal encoders, with UVW signal output
            config
                .encoder
                    .uvwMeasurementPeriod(10)
                    .uvwAverageDepth(2);
        }
    }

    /** Optimizes motor signals to limit unnecessary data over CAN. */
    public static <C extends SparkBaseConfig> void optimizeSignals(C config, boolean hasAbsoluteEncoder, boolean hasExternalOrAlternateEncoder) {
        config
            .signals
                .absoluteEncoderPositionAlwaysOn(hasAbsoluteEncoder)
                .absoluteEncoderVelocityAlwaysOn(hasAbsoluteEncoder)
                .analogPositionAlwaysOn(false) // Uncomment if analog sensors attached to motor controller
                .analogVelocityAlwaysOn(false) // Uncomment if analog sensors attached to motor controller
                .analogVoltageAlwaysOn(false) // Uncomment if analog sensors attached to motor controller
                .externalOrAltEncoderPositionAlwaysOn(hasExternalOrAlternateEncoder)
                .externalOrAltEncoderVelocityAlwaysOn(hasExternalOrAlternateEncoder)
                .faultsAlwaysOn(true)
                .faultsPeriodMs(1000) // faults don't need high update rate
                .iAccumulationAlwaysOn(true)
                .limitsPeriodMs(250) // Uncomment if limit switches attached to motor controller
                .motorTemperaturePeriodMs(1000) // temp doesn't need high update rate
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .warningsAlwaysOn(true)
                .warningsPeriodMs(1000); // warnings don't need high update rate
    }
}