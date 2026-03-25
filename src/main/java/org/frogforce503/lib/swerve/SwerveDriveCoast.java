package org.frogforce503.lib.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

/** Applies coast requests for the drive and steer motors. */
public class SwerveDriveCoast implements SwerveRequest {
    private final CoastOut driveRequest = new CoastOut();
    private final CoastOut steerRequest = new CoastOut();

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        for (SwerveModule<?, ?, ?> module : modulesToApply) {
            module.apply(driveRequest, steerRequest);
        }
        return StatusCode.OK;
    }
}