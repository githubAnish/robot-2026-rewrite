package org.frogforce503.robot.commands;

import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(JoystickUtil.class)
public class ShootFuelIntoHubOrLob extends Command {
    private final Drive drive;
    private final Feeder feeder;
    private final Hood hood;
    private final Flywheels flywheels;
    private final GameViz gameViz;
    private final Supplier<Translation2d> linearVelocitySupplier;

    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            10.0,
            0.0,
            0.5,
            new Constraints(DriveConstants.maxOmega, DriveConstants.maxOmega));

    public ShootFuelIntoHubOrLob(
        Drive drive,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        GameViz gameViz,
        CommandXboxController xboxController
    ) {
        this.drive = drive;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheels = flywheels;
        this.gameViz = gameViz;
        this.linearVelocitySupplier = () -> xboxController.getLinearVelocityFromJoysticks();

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive, feeder, hood, flywheels);
    }

    public ShootFuelIntoHubOrLob(
        Drive drive,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        GameViz gameViz
    ) {
        this.drive = drive;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheels = flywheels;
        this.gameViz = gameViz;
        this.linearVelocitySupplier = Translation2d::new;

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive, feeder, hood, flywheels);
    }

    @Override
    public void initialize() {
        thetaController.reset(
            drive.getAngle().getRadians(),
            drive.getFieldVelocity().omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        // Define shot parameters
        double hoodAngleRad = 0.0;
        double hoodVelocityRadPerSec = 0.0;
        double flywheelsVelocityRadPerSec = 0.0;

        // Get latest shot info
        ShotInfo shotInfo =
            ShotCalculator.getInstance().calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());

        switch (ShotCalculator.getInstance().getShotPreset()) {
            case NONE:
                hoodAngleRad = shotInfo.hoodAngleRad();
                hoodVelocityRadPerSec = shotInfo.hoodVelocityRadPerSec();
                flywheelsVelocityRadPerSec = shotInfo.flywheelsVelocityRadPerSec();
                break;

            case BATTER:
                hoodAngleRad = HoodConstants.BATTER;
                hoodVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.BATTER;
                break;

            case TRENCH:
                hoodAngleRad = HoodConstants.TRENCH;
                hoodVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.TRENCH;
                break;

            case DEPOT:
                hoodAngleRad = HoodConstants.DEPOT;
                hoodVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.DEPOT;
                break;   
        }

        // Aim at target
        aimAtTarget(shotInfo.driveAngle(), shotInfo.driveVelocity());

        // Run subsystems
        hood.setAngle(hoodAngleRad, hoodVelocityRadPerSec);
        flywheels.setVelocity(flywheelsVelocityRadPerSec);

        // Check if shot feasible
        boolean isShotFeasible = ShotCalculator.getInstance().isShotFeasible();

        // Run feeder if shot feasible
        if (isShotFeasible) {
            feeder.setVelocity(FeederConstants.SHOOT);
        }

        // Simulate shooting
        if (RobotBase.isSimulation() && isShotFeasible) {
            gameViz.shootFuel(true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        hood.stop();
        flywheels.stop();
    }

    private void aimAtTarget(Rotation2d driveAngle, double driveVelocity) {
        // Get driver input velocities
        Translation2d driverLinearVelocity = linearVelocitySupplier.get();

        // Calculate speeds
        double xVelocity = driverLinearVelocity.getX() * DriveConstants.maxLinearSpeed;
        double yVelocity = driverLinearVelocity.getY() * DriveConstants.maxLinearSpeed;
        double omega =
            thetaController.calculate(
                drive.getAngle().getRadians(),
                new State(driveAngle.getRadians(), driveVelocity));

        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, omega);

        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                FieldConstants.isRed()
                    ? drive.getAngle().plus(Rotation2d.kPi)
                    : drive.getAngle()));
    }
}