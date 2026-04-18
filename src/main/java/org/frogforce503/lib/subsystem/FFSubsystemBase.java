package org.frogforce503.lib.subsystem;

import org.frogforce503.lib.logging.LoggerUtil;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Wrapper class around WPILib's {@link SubsystemBase} class to create command-based subsystems with common FF boilerplate methods. */
public abstract class FFSubsystemBase extends SubsystemBase {
    protected LoggedNetworkBoolean coastOverride =
        new LoggedNetworkBoolean("Coast Mode/" + getName(), false);

    private boolean inCoast = false;

    protected final Alert coastModeWhileRunning =
        new Alert(getName() + " cannot coast while running! Request ignored.", AlertType.kError);

    @Override
    public void periodic() {
        LoggerUtil.recordCurrentCommand(this);

        // Set coast mode only when disabled and there is a change in the override
        boolean shouldCoast = coastOverride.get();

        if (RobotState.isDisabled() && shouldCoast != inCoast) {
            setBrakeMode(!shouldCoast);
            inCoast = shouldCoast;
        }

        coastModeWhileRunning.set(RobotState.isEnabled() && coastOverride.get());
    };

    protected abstract void setBrakeMode(boolean enabled);
    public abstract void stop();
}