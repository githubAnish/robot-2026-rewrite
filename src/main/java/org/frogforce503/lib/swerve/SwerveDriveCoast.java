package org.frogforce503.lib.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

/** Applies coast requests for the drive and steer motors. */
public class SwerveDriveCoast implements SwerveRequest {
    private final CoastOut m_driveRequest = new CoastOut();
    private final CoastOut m_steerRequest = new CoastOut();

    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        for (SwerveModule<?, ?, ?> module : modulesToApply) {
            module.apply(m_driveRequest, m_steerRequest);
        }
        return StatusCode.OK;
    }
}