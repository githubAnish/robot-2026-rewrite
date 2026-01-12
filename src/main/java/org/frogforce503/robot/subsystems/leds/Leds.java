package org.frogforce503.robot.subsystems.leds;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.logging.LoggerUtil;
import org.frogforce503.robot.subsystems.leds.io.LedsIO;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;

public class Leds extends SubsystemBase {
    private final LedsIO io;
    private final LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();

    @Setter private boolean cameraDisconnected = false;

    public Leds(LedsIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggerUtil.recordCurrentCommand(this);

        io.updateInputs(inputs);
        Logger.processInputs("Leds", inputs);

        if (cameraDisconnected) {
            io.runPattern(LedsRequest.CAMERA_DISCONNECTED);
        }

        // Record cycle time
        LoggedTracer.record("Leds");
    }

    public void stop() {
        if (cameraDisconnected) {
            return;
        }

        io.runPattern(LedsRequest.CLEAR_ANIMATION);
        io.runPattern(LedsRequest.ALL_LEDS_OFF);
    }

    /** Runs the specified LED pattern. See {@link LedsRequest} for available patterns. */
    public void runPattern(ControlRequest pattern) {
        if (cameraDisconnected) {
            return;
        }

        io.runPattern(pattern);
    }
}