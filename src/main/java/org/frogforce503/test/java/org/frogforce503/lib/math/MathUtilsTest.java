package org.frogforce503.test.java.org.frogforce503.lib.math;

import static org.junit.Assert.assertEquals;

import org.frogforce503.lib.math.MathUtils;
import org.junit.Test;

public class MathUtilsTest {
    private final double kTol = 0;

    @Test
    public void testMinOfValues() {
        assertEquals(MathUtils.min(0, 1, 2, 3, 4, 5), 0, kTol);
    }
}