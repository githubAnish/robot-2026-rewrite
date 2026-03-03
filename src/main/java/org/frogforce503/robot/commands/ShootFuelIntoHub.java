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
import org.frogforce503.robot.subsystems.vision.VisionConstants.AprilTagGoal;
import org.ironmaple.simulation.IntakeSimulation;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootFuelIntoHub extends Command {
    private final Drive drive;
    private final Vision vision;

    private final Feeder feeder;
    private final Turret turret;
    private final Hood hood;
    private final Flywheels flywheels;

    // Sim
    private final IntakeSimulation intakeSimulation;

    public ShootFuelIntoHub(
        Drive drive,
        Vision vision,
        Feeder feeder,
        Turret turret,
        Hood hood,
        Flywheels flywheels,
        IntakeSimulation intakeSimulation
    ) {
        this.drive = drive;
        this.vision = vision;

        this.feeder = feeder;
        this.turret = turret;
        this.hood = hood;
        this.flywheels = flywheels;

        this.intakeSimulation = intakeSimulation;

        addRequirements(feeder, flywheels);
    }

    @Override
    public void initialize() {
        vision.setDesiredAprilTagGoal(AprilTagGoal.TURRET_HUB_AIMING);
    }

    @Override
    public void execute() {
        // Define shot params
        double flywheelsVelocityRadPerSec = 0.0;
        boolean isFeasibleShot = false;

        final ShotPreset shotPreset = ShotCalculator.getInstance().getShotPreset();

        // Calculate shot params
        switch (shotPreset) {
            case NONE:
                ShotInfo shotInfo =
                    ShotCalculator.getInstance().calculateHubShotInfo(
                        drive.getPose(),
                        drive.getRobotVelocity(),
                        drive.getFieldVelocity());

                flywheelsVelocityRadPerSec = shotInfo.flywheelsVelocityRadPerSec();
                isFeasibleShot = MathUtils.inRange(shotInfo.turretToTargetDistance(), ShotCalculator.minDistanceHubShoot, ShotCalculator.maxDistanceHubShoot);
                break;

            case BATTER:
                flywheelsVelocityRadPerSec = FlywheelsConstants.BATTER;
                isFeasibleShot = true;
                break;

            case TRENCH:
                flywheelsVelocityRadPerSec = FlywheelsConstants.TRENCH;
                isFeasibleShot = true;
                break;

            case DEPOT:
                flywheelsVelocityRadPerSec = FlywheelsConstants.DEPOT;
                isFeasibleShot = true;
                break;
        }

        // Run subsystems
        flywheels.setVelocity(flywheelsVelocityRadPerSec);
        feeder.setVelocity(FeederConstants.SHOOT);

        // Check if subsystems at setpoint
        boolean turretAtGoal = turret.isAtAngle(turret.getRobotRelativeAngleRad(), TurretConstants.kShootOnMoveTolerance);
        boolean flywheelsAtGoal = flywheels.isAtVelocity(flywheelsVelocityRadPerSec, FlywheelsConstants.kTolerance);
        boolean hoodAtGoal = hood.isAtAngle(hood.getAngleRad(), HoodConstants.kShootOnMoveTolerance);

        isFeasibleShot = isFeasibleShot && turretAtGoal && flywheelsAtGoal && hoodAtGoal;

        ShotCalculator.getInstance().setFeasibleShot(isFeasibleShot);

        // Simulate shooting
        if (RobotBase.isSimulation() && isFeasibleShot) {
            MapleSimUtil.shootFuel(
                drive.getPose(),
                drive.getFieldVelocity(),
                turret.getFieldRelativeAngle(),
                hood.getAngleRad(),
                flywheelsVelocityRadPerSec,
                () -> FieldConstants.Hub.getHubShotPose(),
                intakeSimulation);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        vision.setDesiredAprilTagGoal(AprilTagGoal.GLOBAL_LOCALIZATION);

        feeder.stop();
        flywheels.stop();
    }
}
