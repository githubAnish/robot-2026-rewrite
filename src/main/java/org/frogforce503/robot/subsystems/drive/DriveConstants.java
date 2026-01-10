package org.frogforce503.robot.subsystems.drive;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.lib.swerve.SwervePathController;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.DriveConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class DriveConstants {
    public static final double trackWidthX;
    public static final double trackWidthY;

    public static final double driveBaseRadius;
    public static final Translation2d centerOfRotation = Translation2d.kZero;

    public static final double maxLinearSpeed;
    public static final double maxOmega;

    public static final SwerveDriveKinematics kinematics;
    public static final SwervePathController pathFollower;

    static {
        final DriveConfig driveConfig = Robot.bot.getDriveConfig();

        Translation2d frontLeftModuleTranslation = new Translation2d(driveConfig.frontLeft().LocationX, driveConfig.frontLeft().LocationY);
        Translation2d frontRightModuleTranslation = new Translation2d(driveConfig.frontRight().LocationX, driveConfig.frontRight().LocationY);
        Translation2d backLeftModuleTranslation = new Translation2d(driveConfig.backLeft().LocationX, driveConfig.backLeft().LocationY);
        Translation2d backRightModuleTranslation = new Translation2d(driveConfig.backRight().LocationX, driveConfig.backRight().LocationY);

        trackWidthX = frontLeftModuleTranslation.getDistance(backLeftModuleTranslation);
        trackWidthY = frontLeftModuleTranslation.getDistance(frontRightModuleTranslation);

        driveBaseRadius =
            MathUtils.max(
                frontLeftModuleTranslation.getNorm(),
                frontRightModuleTranslation.getNorm(),
                backLeftModuleTranslation.getNorm(),
                backRightModuleTranslation.getNorm());

        maxLinearSpeed = Robot.bot.getDriveConfig().frontLeft().SpeedAt12Volts;
        maxOmega = maxLinearSpeed / driveBaseRadius;

        kinematics =
            new SwerveDriveKinematics(
                new Translation2d[] {
                    frontLeftModuleTranslation,
                    frontRightModuleTranslation,
                    backLeftModuleTranslation,
                    backRightModuleTranslation});

        final PIDConfig linearPID = Robot.bot.getFollowerLinearPID();
        final PIDConfig thetaPID = Robot.bot.getFollowerThetaPID();

        pathFollower =
            new SwervePathController(
                linearPID.toPIDController(),
                linearPID.toPIDController(),
                thetaPID.toPIDController());
    }
}