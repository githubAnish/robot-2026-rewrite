package org.frogforce503.lib.subsystem;

import org.frogforce503.lib.logging.LoggerUtil;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

/** A thin wrapper around WPILib's {@link SubsystemBase} class to create command-based subsystems with common FF boilerplate methods. */
public abstract class FFSubsystemBase extends SubsystemBase {
    @Getter protected LoggedNetworkBoolean coastOverride =
        new LoggedNetworkBoolean("Coast Mode/" + this.getName(), false);

    protected final Alert coastModeWhileRunning =
        new Alert(this.getName() + " is in coast mode while running!", AlertType.kError);

    private boolean inCoast = false;

    @Override
    public void periodic() {
        LoggerUtil.recordCurrentCommand(this);

        // Set coast mode only when disabled and there is a change in the override
        boolean shouldCoast = coastOverride.get();

        if (RobotState.isDisabled() && shouldCoast != inCoast) {
            inCoast = shouldCoast;
            setBrakeMode(!shouldCoast);
        }

        coastModeWhileRunning
            .set(coastOverride.get() && !RobotState.isDisabled());
    };

    // Actions
    protected abstract void setBrakeMode(boolean enabled);
    public abstract void stop();
}