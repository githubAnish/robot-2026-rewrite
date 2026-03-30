package org.frogforce503.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;

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
public class AimAtHubOrLob extends Command {
    private final Drive drive;
    
    private Supplier<Translation2d> linearVelocitySupplier = Translation2d::new;
    private DoubleSupplier omegaSupplier = () -> 0.0;

    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            15.0,
            0.0,
            0.5,
            new Constraints(DriveConstants.maxOmega, DriveConstants.maxOmega));

    private final double maxDriverOmega = DriveConstants.maxOmega * 0.15;
    private final double translationScalarShootOnMove = 0.25;

    public AimAtHubOrLob(Drive drive) {
        this.drive = drive;

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    public AimAtHubOrLob(Drive drive, CommandXboxController xboxController) {
        this(drive);
        
        this.linearVelocitySupplier = () -> xboxController.getLinearVelocityFromJoysticks();
        this.omegaSupplier = () -> xboxController.getOmegaFromJoysticks();
    }

    @Override
    public void initialize() {
        thetaController.reset(
            drive.getAngle().getRadians(),
            drive.getFieldVelocity().omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        // Get latest shot info
        ShotInfo shotInfo =
            ShotCalculator.getInstance().calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());

        // Get driver input velocities
        Translation2d driverLinearVelocity = linearVelocitySupplier.get();
        double driverOmega = omegaSupplier.getAsDouble();

        // Calculate speeds
        double xVelocity = driverLinearVelocity.getX() * translationScalarShootOnMove * DriveConstants.maxLinearSpeed;
        double yVelocity = driverLinearVelocity.getY() * translationScalarShootOnMove * DriveConstants.maxLinearSpeed;
        double omega =
            thetaController.calculate(
                drive.getAngle().getRadians(),
                new State(shotInfo.driveAngle().getRadians(), shotInfo.driveVelocity()));

        // Fuse driver omega with calculated output
        final double thetaS = Math.abs(driverOmega) * 3.0;
        omega = MathUtil.interpolate(omega, driverOmega * maxDriverOmega, thetaS);

        // Calculate speeds
        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, omega);

        // Apply speeds
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                FieldConstants.isRed()
                    ? drive.getAngle().plus(Rotation2d.kPi)
                    : drive.getAngle()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}