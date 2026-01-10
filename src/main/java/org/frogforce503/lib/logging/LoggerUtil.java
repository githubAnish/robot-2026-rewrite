package org.frogforce503.lib.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class LoggerUtil {
    private LoggerUtil() {}

    public static void recordCurrentCommand(SubsystemBase subsystem) {
        final var currentCommand = subsystem.getCurrentCommand();
        Logger.recordOutput(
            subsystem.getName() + "/Current Command",
            currentCommand == null ? "None" : currentCommand.getName());
    }   
}