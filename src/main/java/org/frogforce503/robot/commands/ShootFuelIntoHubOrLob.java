package org.frogforce503.robot.commands;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.rebuilt.MapleSimUtil;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.ShotPreset;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.frogforce503.robot.subsystems.superstructure.turret.Turret;
import org.frogforce503.robot.subsystems.superstructure.turret.TurretConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootFuelIntoHubOrLob extends Command {
    private final Drive drive;

    private final Feeder feeder;
    private final Turret turret;
    private final Hood hood;
    private final Flywheels flywheels;

    // Sim
    private final IntakeSimulation intakeSimulation;

    public ShootFuelIntoHubOrLob(
        Drive drive,
        Vision vision,
        Feeder feeder,
        Turret turret,
        Hood hood,
        Flywheels flywheels,
        IntakeSimulation intakeSimulation
    ) {
        this.drive = drive;

        this.feeder = feeder;
        this.turret = turret;
        this.hood = hood;
        this.flywheels = flywheels;

        this.intakeSimulation = intakeSimulation;

        addRequirements(feeder, flywheels);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        final boolean isHubShot = FieldConstants.inAllianceZone(drive.getPose()); // For checking whether to compute hub or lob shot info

        // Define shot params
        Rotation2d turretFieldRelativeAngle = Rotation2d.kZero;
        double hoodAngleRad = 0.0;
        double flywheelsVelocityRadPerSec = 0.0;
        boolean isFeasibleShot = false;

        final ShotPreset shotPreset = ShotCalculator.getInstance().getShotPreset();

        // Calculate shot params
        switch (shotPreset) {
            case NONE:
                ShotInfo shotInfo =
                    isHubShot
                        ? ShotCalculator.getInstance().calculateHubShotInfo(
                            drive.getPose(),
                            drive.getRobotVelocity(),
                            drive.getFieldVelocity())

                        : ShotCalculator.getInstance().calculateLobShotInfo(
                            drive.getPose(),
                            drive.getRobotVelocity(),
                            drive.getFieldVelocity());

                turretFieldRelativeAngle = shotInfo.turretFieldRelativeAngle();
                hoodAngleRad = shotInfo.hoodAngleRad();
                flywheelsVelocityRadPerSec = shotInfo.flywheelsVelocityRadPerSec();
                isFeasibleShot =
                    isHubShot
                        ? MathUtils.inRange(shotInfo.turretToTargetDistance(), ShotCalculator.minDistanceHubShoot, ShotCalculator.maxDistanceHubShoot)
                        : MathUtils.inRange(shotInfo.turretToTargetDistance(), ShotCalculator.minDistanceLobShoot, ShotCalculator.maxDistanceLobShoot);
                break;

            case BATTER:
                turretFieldRelativeAngle = TurretConstants.BATTER_FIELD_RELATIVE;
                hoodAngleRad = HoodConstants.BATTER;
                flywheelsVelocityRadPerSec = FlywheelsConstants.BATTER;
                isFeasibleShot = true;
                break;

            case TRENCH:
                turretFieldRelativeAngle = TurretConstants.TRENCH_FIELD_RELATIVE;
                hoodAngleRad = HoodConstants.TRENCH;
                flywheelsVelocityRadPerSec = FlywheelsConstants.TRENCH;
                isFeasibleShot = true;
                break;

            case DEPOT:
                turretFieldRelativeAngle = TurretConstants.DEPOT_FIELD_RELATIVE;
                hoodAngleRad = HoodConstants.DEPOT;
                flywheelsVelocityRadPerSec = FlywheelsConstants.DEPOT;
                isFeasibleShot = true;
                break;
        }

        // Run subsystems
        feeder.setVelocity(FeederConstants.SHOOT);
        flywheels.setVelocity(flywheelsVelocityRadPerSec);

        // Check if subsystems at setpoint
        boolean turretAtGoal = turret.isAtAngle(turretFieldRelativeAngle, TurretConstants.kShootOnMoveTolerance);
        boolean hoodAtGoal = hood.isAtAngle(hoodAngleRad, HoodConstants.kShootOnMoveTolerance);
        boolean flywheelsAtGoal = flywheels.isAtVelocity(flywheelsVelocityRadPerSec, FlywheelsConstants.kTolerance);

        isFeasibleShot = isFeasibleShot && turretAtGoal && hoodAtGoal && flywheelsAtGoal;

        ShotCalculator.getInstance().setFeasibleShot(isFeasibleShot);

        // Simulate shooting
        if (RobotBase.isSimulation() && isFeasibleShot) {
            MapleSimUtil.shootFuel(
                drive.getPose(),
                drive.getFieldVelocity(),
                turret.getFieldRelativeAngle(),
                hood.getAngleRad(),
                flywheelsVelocityRadPerSec,
                () ->
                    isHubShot
                        ? FieldConstants.Hub.getHubShotPose()
                        : new Translation3d(FieldConstants.Depot.getLobShotPose()),
                intakeSimulation,
                true);
        }
        
        // Log data
        Logger.recordOutput("ShootFuelIntoHubOrLob/Is Hub Shot?", isHubShot);
        Logger.recordOutput("ShootFuelIntoHubOrLob/Turret At Goal?", turretAtGoal);
        Logger.recordOutput("ShootFuelIntoHubOrLob/Hood At Goal?", hoodAtGoal);
        Logger.recordOutput("ShootFuelIntoHubOrLob/Flywheels At Goal?", flywheelsAtGoal);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        flywheels.stop();
    }
}
