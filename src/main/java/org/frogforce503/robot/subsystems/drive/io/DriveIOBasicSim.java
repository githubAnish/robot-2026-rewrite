package org.frogforce503.robot.subsystems.drive.io;

import org.frogforce503.robot.subsystems.drive.DriveConstants;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class DriveIOBasicSim implements DriveIO {
    // Constants
    private final double theoreticalWheelRadiusInches = 2.00; // Inches

    // State
    private SwerveModuleState[] moduleStates =
        new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()};

    private Pose2d currentPose = Pose2d.kZero;
    private ChassisSpeeds currentVelocity = new ChassisSpeeds();

    private double lastUpdatedTime = -1.0;
    private double dt = 0;

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        SwerveDriveState currentState = getCurrentState();

        inputs.fromSwerveDriveState(currentState);

        update();
        
        // Calculate characterization data
        for (int i = 0; i < 4; i++) {
            double wheelSpeedMetersPerSec = moduleStates[i].speedMetersPerSecond;

            inputs.driveVelocitiesRadPerSec[i] = wheelSpeedMetersPerSec / theoreticalWheelRadiusInches;
            inputs.drivePositionsRad[i] += inputs.driveVelocitiesRadPerSec[i] * dt;
        }
    }

    @Override
    public void setPose(Pose2d pose) {
        currentPose = pose;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        currentPose = new Pose2d(currentPose.getTranslation(), angle);
    }

    @Override
    public void brake() {
        currentVelocity = new ChassisSpeeds();
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds) {
        currentVelocity = speeds;
        moduleStates = DriveConstants.kinematics.toSwerveModuleStates(currentVelocity);
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {
        runVelocity(speeds); // Ignore module force feedforwards in simulation
    }

    public void update() {
        double currentTime = Timer.getFPGATimestamp();

        if (lastUpdatedTime > 0) {
            dt = currentTime - lastUpdatedTime;

            currentPose = currentPose.exp(currentVelocity.toTwist2d(dt));
        }
        
        lastUpdatedTime = currentTime;
    }

    private SwerveDriveState getCurrentState() {
        SwerveDriveState currentState = new SwerveDriveState();
        
        currentState.SuccessfulDaqs = 0;
        currentState.FailedDaqs = 0;
        currentState.Pose = currentPose;
        currentState.ModuleStates = moduleStates;
        currentState.OdometryPeriod = 0.02;

        return currentState;
    }  
}
