package org.frogforce503.robot.subsystems.drive;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    // Hardware / Configuration
    public static final SwerveDrivetrainConstants drivetrainConstants = TunerConstants.DrivetrainConstants;
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeft = TunerConstants.FrontLeft;
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRight = TunerConstants.FrontRight;
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeft = TunerConstants.BackLeft;
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRight = TunerConstants.BackRight;

    public static final double trackWidthX;
    public static final double trackWidthY;
    public static final double bumperLength = Units.inchesToMeters(30);
    public static final double bumperWidth = Units.inchesToMeters(35);

    public static final double driveBaseRadius;
    public static final Translation2d centerOfRotation = Translation2d.kZero;

    public static final double maxLinearSpeed;
    public static final double maxOmega;

    public static final double wheelCOF = 1.9;

    public static final Translation2d[] moduleTranslations;

    // Swerve Control
    public static final PIDConfig pathplannerLinearPID = new PIDConfig(0.25, 0, 0.1);
    public static final PIDConfig pathplannerThetaPID = new PIDConfig(3, 0, 0);
    public static final PathConstraints pathplannerConstraints;

    public static final PIDConfig blineLinearPID = new PIDConfig(5, 0, 0.5);
    public static final PIDConfig blineThetaPID = new PIDConfig(3, 0, 0);
    public static final PIDConfig blineCtePID = new PIDConfig(2, 0, 0);

    public static final double aimTolerance = Units.degreesToRadians(2.5);

    static {
        // Hardware / Configuration
        Translation2d frontLeftModuleTranslation = new Translation2d(frontLeft.LocationX, frontLeft.LocationY);
        Translation2d frontRightModuleTranslation = new Translation2d(frontRight.LocationX, frontRight.LocationY);
        Translation2d backLeftModuleTranslation = new Translation2d(backLeft.LocationX, backLeft.LocationY);
        Translation2d backRightModuleTranslation = new Translation2d(backRight.LocationX, backRight.LocationY);

        trackWidthX = frontLeftModuleTranslation.getDistance(backLeftModuleTranslation);
        trackWidthY = frontLeftModuleTranslation.getDistance(frontRightModuleTranslation);

        driveBaseRadius =
            MathUtils.max(
                frontLeftModuleTranslation.getNorm(),
                frontRightModuleTranslation.getNorm(),
                backLeftModuleTranslation.getNorm(),
                backRightModuleTranslation.getNorm());

        maxLinearSpeed = frontLeft.SpeedAt12Volts;
        maxOmega = maxLinearSpeed / driveBaseRadius;

        moduleTranslations =
            new Translation2d[] {
                frontLeftModuleTranslation,
                frontRightModuleTranslation,
                backLeftModuleTranslation,
                backRightModuleTranslation};

        // Swerve Control
        pathplannerConstraints = new PathConstraints(maxLinearSpeed, maxLinearSpeed * 0.7, maxOmega, maxOmega * 0.7);
    }
}