package org.frogforce503.robot.subsystems.superstructure;

import java.util.function.Supplier;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.VirtualSubsystem;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
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
    @Getter private final IntakePivot intakePivot;
    @Getter private final IntakeRoller intakeRoller;
    @Getter private final Indexer indexer;
    @Getter private final Feeder feeder;
    @Getter private final Turret turret;
    @Getter private final Flywheels flywheels;
    @Getter private final Hood hood;

    // Inputs
    @Setter @Getter private ShotPreset shotPreset = ShotPreset.NONE;
    @Setter @Getter private boolean feasibleShot;

    // Viz
    @Getter private final SuperstructureViz viz;

    // Overrides
    private LoggedNetworkBoolean superstructureCoastOverride =
        new LoggedNetworkBoolean("Coast Mode/Superstructure", false);

    private boolean inCoast = false;

    public Superstructure(
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Indexer indexer,
        Feeder feeder,
        Turret turret,
        Flywheels flywheels,
        Hood hood,
        Supplier<Pose2d> robotPoseSupplier
    ) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.indexer = indexer;
        this.feeder = feeder;
        this.turret = turret;
        this.flywheels = flywheels;
        this.hood = hood;

        this.viz = new SuperstructureViz(robotPoseSupplier);
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
                turret.getAngleRad(),
                hood.getAngleRad());
        }

        Logger.recordOutput("Superstructure/ShotPreset", shotPreset);
        Logger.recordOutput("Superstructure/Is Shot Feasible?", feasibleShot);

        // Record cycle time
        LoggedTracer.record("Superstructure");
    }

    public boolean isFull() {
        return indexer.isCompressed();
    }

    // Actions
    public void setCoastMode(boolean enabled) {
        intakePivot.getCoastOverride().set(enabled);
        intakeRoller.getCoastOverride().set(enabled);
        indexer.getCoastOverride().set(enabled);
        feeder.getCoastOverride().set(enabled);
        turret.getCoastOverride().set(enabled);
        flywheels.getCoastOverride().set(enabled);
        hood.getCoastOverride().set(enabled);
    }

    public void seedTurretRelativePosition() {
        turret.seedRelativePosition();
    }

    public void stop() {
        intakePivot.stop();
        intakeRoller.stop();
        indexer.stop();
        feeder.stop();
        turret.stop();
        flywheels.stop();
        hood.stop();
    }
}