package org.frogforce503.lib.math;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;

/**
 * Miscellaneous math utility functions.
 * 
 * @see {@link MathUtil} for more common math utility functions.
 */
public final class MathUtils {
    private MathUtils() {}

    /** Finds min of {@code values}. */
    public static double min(double... values) {
        return Arrays.stream(values).min().orElse(Double.MAX_VALUE);
    }

    /** Finds max of {@code values}. */
    public static double max(double... values) {
        return Arrays.stream(values).max().orElse(Double.MIN_VALUE);
    }

    /**
     * Returns if the value is in the range [lowerBound, upperBound].
     *
     * @param lowerBound The lower bound of the range.
     * @param upperBound The upper bound of the range.
     * @param value The value.
     * @return If the value is in the range.
     */
    public static boolean inRange(double value, double lowerBound, double upperBound) {
        return lowerBound <= value && value <= upperBound;
    }
}