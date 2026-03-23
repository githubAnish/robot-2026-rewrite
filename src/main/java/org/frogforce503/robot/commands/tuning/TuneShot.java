package org.frogforce503.robot.commands.tuning;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.commands.TrackTargetCommand;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/** Tunes hood angle and flywheels speed for a specific distance or preset. */
public class TuneShot extends TrackTargetCommand {
    private final GameViz gameViz;
    private final BooleanSupplier isShootingSupplier;

    private final LoggedTunableNumber hoodAngleDeg =
        new LoggedTunableNumber("TuneShot/HoodAngleDeg", Units.radiansToDegrees(HoodConstants.START));

    private final LoggedTunableNumber flywheelsVelocityRpm =
        new LoggedTunableNumber("TuneShot/FlywheelsVelocityRpm", Units.radiansPerSecondToRotationsPerMinute(FlywheelsConstants.START));

    private final LoggedNetworkBoolean recordShot =
        new LoggedNetworkBoolean("Tuning/TuneShot/Record Shot?", false);

    private String hoodMapEntries = "";
    private String flywheelsMapEntries = "";

    public TuneShot(
        Drive drive,
        Vision vision,
        Turret turret,
        Hood hood,
        Flywheels flywheels,
        GameViz gameViz,
        BooleanSupplier isShootingSupplier
    ) {
        super(drive, vision, turret, hood, flywheels, isShootingSupplier);

        this.gameViz = gameViz;
        this.isShootingSupplier = isShootingSupplier;
    }

    @Override
    public void initialize() {
        super.initialize();

        flywheelsVelocityRpm.setTuningMode(true);
        hoodAngleDeg.setTuningMode(true);
    }

    @Override
    public void execute() {
        super.setHoodAngleOverride(OptionalDouble.of(Units.degreesToRadians(hoodAngleDeg.get())));
        super.setFlywheelsVelocityOverride(OptionalDouble.of(Units.rotationsPerMinuteToRadiansPerSecond(flywheelsVelocityRpm.get())));

        super.execute(); // Calculate shot params after override applied

        // Simulate shooting
        if (RobotBase.isSimulation() && isShootingSupplier.getAsBoolean()) {
            gameViz.shootFuel(false);
        }

        // Record shot
        recordShot();
    }

    private void recordShot() {
        if (!recordShot.get()) {
            return;
        }

        final String prefix = super.isTrackingHub() ? "hub" : "lob";

        hoodMapEntries +=
            prefix + "HoodAngleMap.put(" + super.getTurretToTargetDistance() + ", Units.degreesToRadians(" + hoodAngleDeg.get() + "));\n";

        flywheelsMapEntries +=
            prefix + "FlywheelSpeedMap.put(" + super.getTurretToTargetDistance() + ", Units.rotationsPerMinuteToRadiansPerSecond(" + flywheelsVelocityRpm.get() + "));\n";

        Logger.recordOutput("TuneShot/Shot Info (for map)", hoodMapEntries + "\n" + flywheelsMapEntries);

        recordShot.set(false);
    }
}