package org.frogforce503.lib.swerve;

import org.frogforce503.robot.subsystems.drive.DriveConstants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Sets the swerve drive module states to point on the robot in an "O"
 * fashion, creating a natural brake resisting translational motion and allowing rotational motion.
 */
public class SwerveDriveBrakeO implements SwerveRequest {
    private final ModuleRequest moduleRequest =
        new ModuleRequest()
            .withDriveRequest(DriveRequestType.OpenLoopVoltage)
            .withSteerRequest(SteerRequestType.Position);

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        for (int i = 0; i < modulesToApply.length; i++) {
            modulesToApply[i].apply(
                moduleRequest
                    .withState(
                        new SwerveModuleState(
                            0,
                            DriveConstants.moduleTranslations[i].getAngle().plus(Rotation2d.kCW_Pi_2))));
        }
        for (SwerveModule<?, ?, ?> module : modulesToApply) {
            module.apply(moduleRequest);
        }
        return StatusCode.OK;
    }
}