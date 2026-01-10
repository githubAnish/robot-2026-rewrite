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

    /** Finds the minimum of {@code values}. */
    public static double min(double... values) {
        return
            Arrays
                .stream(values)
                .min()
                .orElse(Double.MAX_VALUE);
    }

    /** Finds max of {@code values} */
    public static double max(double... values) {
        return
            Arrays
                .stream(values)
                .max()
                .orElse(Double.MIN_VALUE);
    }

    /** Rounds a number to a specified decimal places. */
    public static double roundTo(double num, double digits) {
        if (digits <= 0) {
            throw new IllegalArgumentException("Digits must be only positive.");
        }

        double scalar = Math.pow(10, digits);
        return Math.round(num * scalar) / scalar;
    }

    public static boolean isIn(double toCheckValue, double... options) {
        return
            Arrays
                .stream(options)
                .anyMatch(v -> v == toCheckValue);
    }

    /**
     * Returns if the value is in the range [lowerBound, upperBound].
     *
     * @param lowerBound The lower bound of the range.
     * @param upperBound The upper bound of the range.
     * @param value      The value.
     * @return If the value is in the range.
     */
    public static boolean inRange(double value, double lowerBound, double upperBound) {
        return lowerBound <= value && value <= upperBound;
    }

    /**
     * Solves the equation {@code 0 = ax² + bx + c} for x and
     * returns the real results.
     *
     * @param a the a coefficient
     * @param b the b coefficient
     * @param c the c coefficient
     * @return the real roots of the equation
     */
    public static double[] quadratic(double a, double b, double c) {
        double discriminant = Math.sqrt(b * b - 4 * a * c);
        
        if (Double.isNaN(discriminant)) {
            return new double[0]; // No roots
        } else if (discriminant == 0) {
            return new double[] { -b / (2 * a) }; // One root
        }

        return
            new double[] {
                (-b + discriminant) / (2 * a),
                (-b - discriminant) / (2 * a)};
    }
}
