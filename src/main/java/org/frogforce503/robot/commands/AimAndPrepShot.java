package org.frogforce503.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.robot.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.feeder.Feeder;
import org.frogforce503.robot.subsystems.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.launcher.LaunchCalculator;
import org.frogforce503.robot.subsystems.launcher.LaunchCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.launcher.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.launcher.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.launcher.hood.Hood;
import org.frogforce503.robot.subsystems.launcher.hood.HoodConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(JoystickUtil.class)
public class AimAndPrepShot extends Command {
    private final Drive drive;
    private final Feeder feeder;
    private final Hood hood;
    private final Flywheels flywheels;
    
    private Supplier<Translation2d> linearVelocitySupplier = Translation2d::new;
    private DoubleSupplier omegaSupplier = () -> 0.0;

    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            10.0,
            0.0,
            0.5,
            new Constraints(DriveConstants.maxOmega, DriveConstants.maxOmega));

    private final double maxDriverOmega = DriveConstants.maxOmega * 0.15;
    private final double translationScalarShootOnMove = 0.2;

    public AimAndPrepShot(
        Drive drive,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels
    ) {
        this.drive = drive;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheels = flywheels;

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive, feeder, hood, flywheels);
    }

    public AimAndPrepShot(
        Drive drive,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        CommandXboxController xboxController
    ) {
        this(drive, feeder, hood, flywheels);
        
        this.linearVelocitySupplier = () -> xboxController.getLinearVelocityFromJoysticks();
        this.omegaSupplier = () -> xboxController.getOmegaFromJoysticks();
    }

    @Override
    public void initialize() {
        thetaController.reset(
            drive.getRotation().getRadians(),
            drive.getFieldVelocity().omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        // Get latest shot info
        ShotInfo shotInfo =
            LaunchCalculator.getInstance().calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());
        
        // Prep & aim for shot
        prepShot(shotInfo);
        aimAtTarget(shotInfo);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        feeder.stop();
        hood.stop();
        flywheels.stop();
    }

    private void prepShot(ShotInfo shotInfo) {
        // Define shot parameters
        double hoodAngleRad = 0.0;
        double hoodVelocityRadPerSec = 0.0;
        double flywheelsVelocityRadPerSec = 0.0;

        switch (LaunchCalculator.getInstance().getShotPreset()) {
            case NONE:
                hoodAngleRad = shotInfo.hoodAngleRad();
                hoodVelocityRadPerSec = shotInfo.hoodVelocityRadPerSec();
                flywheelsVelocityRadPerSec = shotInfo.flywheelsVelocityRadPerSec();
                break;

            case BATTER:
                hoodAngleRad = HoodConstants.BATTER;
                flywheelsVelocityRadPerSec = FlywheelsConstants.BATTER;
                break;

            case TRENCH:
                hoodAngleRad = HoodConstants.TRENCH;
                flywheelsVelocityRadPerSec = FlywheelsConstants.TRENCH;
                break;

            case TOWER:
                hoodAngleRad = HoodConstants.TOWER;
                flywheelsVelocityRadPerSec = FlywheelsConstants.TOWER;
                break;   
        }

        // Run subsystems
        feeder.setVelocity(FeederConstants.SHOOT);
        hood.setAngle(hoodAngleRad, hoodVelocityRadPerSec);
        flywheels.setVelocity(flywheelsVelocityRadPerSec);
    }

    private void aimAtTarget(ShotInfo shotInfo) {
        // Get driver input velocities
        Translation2d driverLinearVelocity = linearVelocitySupplier.get();
        double driverOmega = omegaSupplier.getAsDouble();

        // Calculate speeds
        Translation2d linearVelocity = driverLinearVelocity.times(translationScalarShootOnMove * DriveConstants.maxLinearSpeed);
        double omega =
            thetaController.calculate(
                drive.getRotation().getRadians(),
                new State(shotInfo.driveAngle().getRadians(), shotInfo.driveVelocity()));

        // Fuse driver omega with calculated output
        final double thetaS = Math.abs(driverOmega) * 3.0;
        omega = MathUtil.interpolate(omega, driverOmega * maxDriverOmega, thetaS);

        ChassisSpeeds speeds =
            new ChassisSpeeds(
                linearVelocity.getX(),
                linearVelocity.getY(),
                omega);

        // Apply speeds
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                FieldConstants.isRed()
                    ? drive.getRotation().plus(Rotation2d.kPi)
                    : drive.getRotation()));
    }
}