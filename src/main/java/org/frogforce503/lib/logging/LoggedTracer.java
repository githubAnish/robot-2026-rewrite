package org.frogforce503.lib.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

/** Utility class for logging code execution times. */
public final class LoggedTracer {
    private static double startTime = -1.0;

    private LoggedTracer() {}

    /** Reset the clock. */
    public static void reset() {
        startTime = Timer.getFPGATimestamp();
    }

    /** Save the time elapsed since the last reset or record. */
    public static void record(String epochName) {
        double now = Timer.getFPGATimestamp();
        Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - startTime) * 1000.0);
        startTime = now;
    }
}