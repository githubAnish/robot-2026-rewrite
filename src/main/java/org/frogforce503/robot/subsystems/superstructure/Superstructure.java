package org.frogforce503.robot.subsystems.superstructure;

import java.util.function.Supplier;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

public class Superstructure extends VirtualSubsystem {
    // Subsystems

    // Inputs

    @Setter @Getter private SuperstructureMode currentMode = SuperstructureMode.NONE;

    // Viz
    @Getter private final SuperstructureViz viz;

    // Overrides
    private LoggedNetworkBoolean superstructureCoastOverride =
        new LoggedNetworkBoolean("Coast Mode/Superstructure", false);

    private boolean inCoast = false;

    public Superstructure(
        Supplier<Pose2d> robotPoseSupplier
    ) {
        this.viz = new SuperstructureViz(this, robotPoseSupplier);
    }

    @Override
    public void periodic() {
        boolean shouldCoast = superstructureCoastOverride.get();
        
        if (RobotState.isDisabled() && shouldCoast != inCoast) {
            inCoast = shouldCoast;
            setCoastMode(shouldCoast);
        }

        // Update viz
        if (RobotBase.isSimulation()) {
            viz.update();
        }

        Logger.recordOutput("Superstructure/Mode", currentMode);

        // Record cycle time
        LoggedTracer.record("Superstructure");
    }

    // Actions
    public void setCoastMode(boolean enabled) {
        
    }

    public void stop() {
        
    }
}