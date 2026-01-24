package org.frogforce503.lib.io;

import org.frogforce503.lib.math.GeomUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class JoystickUtil {
    private static final double kDeadband = 0.2;

    private JoystickUtil() {}

    public static Translation2d getLinearVelocityFromJoysticks(CommandXboxController xboxController) {
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

    public static double getOmegaFromJoysticks(CommandXboxController xboxController) {
        // Get joystick input
        double driverOmega = -xboxController.getRightX();

        // Apply deadband
        double omega = MathUtil.applyDeadband(driverOmega, kDeadband);

        // Square magnitude for more precise control & return new angular velocity
        return Math.copySign(omega * omega, omega);
    }
}
