package org.frogforce503.robot.subsystems.drive;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.lib.swerve.SwervePathController;
import org.frogforce503.robot.constants.tuner.TunerConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

    public static final double driveBaseRadius;
    public static final Translation2d centerOfRotation = Translation2d.kZero;

    public static final double maxLinearSpeed;
    public static final double maxOmega;

    public static final double mass = Units.lbsToKilograms(87.4200862); // from CAD
    public static final double wheelCOF = 1.9;
    public static final double bumperLength = Units.inchesToMeters(30);
    public static final double bumperWidth = Units.inchesToMeters(35);

    // Swerve Control
    public static final SwerveDriveKinematics kinematics;
    public static final SwervePathController pathFollower;

    public static final PIDConfig linearPID = new PIDConfig(0.25, 0.0, 0.1);
    public static final PIDConfig thetaPID = new PIDConfig(3.0, 0.0, 0.0);

    public static final double aimTolerance = Units.degreesToRadians(5);

    static {
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

        kinematics =
            new SwerveDriveKinematics(
                new Translation2d[] {
                    frontLeftModuleTranslation,
                    frontRightModuleTranslation,
                    backLeftModuleTranslation,
                    backRightModuleTranslation});

        pathFollower =
            new SwervePathController(
                linearPID.toPIDController(),
                linearPID.toPIDController(),
                thetaPID.toPIDController());
    }
}