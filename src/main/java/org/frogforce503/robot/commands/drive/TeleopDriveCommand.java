package org.frogforce503.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.FieldInfo;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;

// Notes:
// Need to figure out best values for rate limiters, heading hold (aka stabilization), and any feedforward assists if needed
public class TeleopDriveCommand extends Command {
    // Requirements
    private final Drive drive;
    private final CommandXboxController xboxController;
    private final BooleanSupplier robotRelative;
    private final BooleanSupplier slowMode;
    private final DoubleSupplier teleopTranslationScalar;
    private final DoubleSupplier teleopRotationScalar;
    private final double kDeadband = 0.2;

    // private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    // private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    // private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(3.0);

    public TeleopDriveCommand(Drive drive, CommandXboxController xboxController) {
        this.drive = drive;
        this.xboxController = xboxController;
        this.robotRelative = drive::isRobotRelative;
        this.slowMode = drive::isSlowMode;
        this.teleopTranslationScalar = drive::getTeleopTranslationScalar;
        this.teleopRotationScalar = drive::getTeleopRotationScalar;

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Get driver input velocities
        Translation2d driverLinearVelocity = getLinearVelocityFromJoysticks();
        double driverOmega = getOmegaFromJoysticks();

        // Apply slow mode if necessary
        drive.setTeleopTranslationScalar(slowMode.getAsBoolean() ? 0.25 : 1.0);
        drive.setTeleopRotationScalar(slowMode.getAsBoolean() ? 0.25 : 1.0);

        // Calculate speeds
        double xVelocity = driverLinearVelocity.getX() * DriveConstants.maxLinearSpeed * MathUtil.clamp(teleopTranslationScalar.getAsDouble(), 0, 1);
        double yVelocity = driverLinearVelocity.getY() * DriveConstants.maxLinearSpeed * MathUtil.clamp(teleopTranslationScalar.getAsDouble(), 0, 1);
        double omega = driverOmega * DriveConstants.maxOmega * MathUtil.clamp(teleopRotationScalar.getAsDouble(), 0, 1);

        // Apply speeds to drivetrain
        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, omega);

        drive.runVelocity(
            robotRelative.getAsBoolean()
                ? speeds
                : ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    FieldInfo.isRed()
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

    public Translation2d getLinearVelocityFromJoysticks() {
        // Get joystick input
        double driverX = -xboxController.getLeftY();
        double driverY = -xboxController.getLeftX();

        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(driverX, driverY), kDeadband);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(driverY, driverX));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return
            GeomUtil
                .toPose2d(linearDirection)
                .transformBy(GeomUtil.toTransform2d(linearMagnitude, 0.0))
                .getTranslation();
    }

    public double getOmegaFromJoysticks() {
        // Get joystick input
        double driverOmega = -xboxController.getRightX();

        // Apply deadband
        double omega = MathUtil.applyDeadband(driverOmega, kDeadband);

        // Square magnitude for more precise control & return new angular velocity
        return Math.copySign(omega * omega, omega);
    }
}