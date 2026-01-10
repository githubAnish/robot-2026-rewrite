package org.frogforce503.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;

public class DriveRequest {
    public static final ApplyRobotSpeeds APPLY_ROBOT_SPEEDS =
        new ApplyRobotSpeeds()
            .withCenterOfRotation(DriveConstants.centerOfRotation)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDesaturateWheelSpeeds(true);

    public static final RobotCentricFacingAngle ROBOT_CENTRIC_FACING_ANGLE =
        new RobotCentricFacingAngle()
            .withCenterOfRotation(DriveConstants.centerOfRotation)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDesaturateWheelSpeeds(true);

    public static final FieldCentric FIELD_CENTRIC =
        new FieldCentric()
            .withCenterOfRotation(DriveConstants.centerOfRotation)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDesaturateWheelSpeeds(true);

    public static final SysIdSwerveTranslation RUN_CHARACTERIZATION = new SysIdSwerveTranslation();
}
