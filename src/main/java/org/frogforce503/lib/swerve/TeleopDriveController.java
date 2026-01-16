package org.frogforce503.lib.swerve;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.FieldInfo;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Getter;
import lombok.Setter;

// Notes:
// Need to figure out best values for rate limiters, heading hold (aka stabilization), and any feedforward assists if needed
public class TeleopDriveController {
    // Requirements
    private final Drive drive;
    private final CommandXboxController xboxController;

    // Constants
    private final double kDeadband = 0.2;

    // private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    // private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    // private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(3.0);

    // State
    @Setter @Getter private double teleopTranslationScalar = 1.0;
    @Setter @Getter private double teleopRotationScalar = 1.0;
    
    public TeleopDriveController(Drive drive, CommandXboxController xboxController) {
        this.drive = drive;
        this.xboxController = xboxController;
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

    public void update() {
        // Get driver input velocities
        Translation2d driverLinearVelocity = getLinearVelocityFromJoysticks();
        double driverOmega = getOmegaFromJoysticks();

        // Calculate speeds
        double xVelocity = driverLinearVelocity.getX() * DriveConstants.maxLinearSpeed * MathUtil.clamp(teleopTranslationScalar, 0, 1);
        double yVelocity = driverLinearVelocity.getY() * DriveConstants.maxLinearSpeed * MathUtil.clamp(teleopTranslationScalar, 0, 1);
        double omega = driverOmega * DriveConstants.maxOmega * MathUtil.clamp(teleopRotationScalar, 0, 1);

        // Apply speeds to drivetrain
        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, omega);

        drive.runVelocity(
            drive.isRobotRelative()
                ? speeds
                : ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    FieldInfo.isRed()
                        ? drive.getAngle().plus(Rotation2d.kPi)
                        : drive.getAngle()));
    }
}