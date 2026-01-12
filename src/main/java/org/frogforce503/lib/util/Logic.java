package org.frogforce503.lib.util;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

/** Utility class to handle BooleanSuppliers. */
public final class Logic {
    private Logic() {}

    public static BooleanSupplier and(BooleanSupplier... conditions) {
        return
            () ->
                Arrays
                    .stream(conditions)
                    .allMatch(BooleanSupplier::getAsBoolean);
    }

    public static BooleanSupplier or(BooleanSupplier... conditions) {
        return
            () ->
                Arrays
                    .stream(conditions)
                    .anyMatch(BooleanSupplier::getAsBoolean);
    }

    public static BooleanSupplier not(BooleanSupplier condition) {
        return
            () ->
                !condition.getAsBoolean();
    }

    public static BooleanSupplier nor(BooleanSupplier... conditions) {
        return not(or(conditions));
    }
}
