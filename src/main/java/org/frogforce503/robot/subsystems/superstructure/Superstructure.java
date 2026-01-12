package org.frogforce503.robot.subsystems.superstructure;

import java.util.function.Supplier;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.VirtualSubsystem;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

// Notes:
// assume no turret, until block cad comes out or more talk comes out or most of this code is done
public class Superstructure extends VirtualSubsystem {
    // Subsystems
    private final IntakeRoller intakeRoller;
    private final Flywheels flywheels;
    private final Hood hood;

    // Inputs
    @Setter @Getter private ShotPreset shotPreset = ShotPreset.NONE;

    // Viz
    @Getter private final SuperstructureViz viz;

    // Overrides
    private LoggedNetworkBoolean superstructureCoastOverride =
        new LoggedNetworkBoolean("Coast Mode/Superstructure", false);

    private boolean inCoast = false;

    public Superstructure(
        IntakeRoller intakeRoller,
        Flywheels flywheels,
        Hood hood,
        Supplier<Pose2d> robotPoseSupplier
    ) {
        this.intakeRoller = intakeRoller;
        this.flywheels = flywheels;
        this.hood = hood;

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
            viz.update(
                intakeRoller.getVelocityRadPerSec(),
                flywheels.getVelocityRadPerSec(),
                hood.getAngleRad());
        }

        Logger.recordOutput("Superstructure/ShotPreset", shotPreset);

        // Record cycle time
        LoggedTracer.record("Superstructure");
    }

    // Actions
    public void setCoastMode(boolean enabled) {
        intakeRoller.getCoastOverride().set(enabled);
        flywheels.getCoastOverride().set(enabled);
        hood.getCoastOverride().set(enabled);
    }

    public void stop() {
        intakeRoller.stop();
        flywheels.stop();
        hood.stop();
    }
}