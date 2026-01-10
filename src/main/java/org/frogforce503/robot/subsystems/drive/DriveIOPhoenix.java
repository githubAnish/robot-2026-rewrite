package org.frogforce503.robot.subsystems.drive;

import org.frogforce503.lib.swerve.SwerveDriveCoast;
import org.frogforce503.robot.Robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class DriveIOPhoenix extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {
    private final BaseStatusSignal[] drivePositionSignals = new BaseStatusSignal[4];
    private final BaseStatusSignal[] driveVelocitySignals = new BaseStatusSignal[4];
    private final StatusSignal<Angle> gyroYaw;

    public DriveIOPhoenix(SwerveModuleConstants<?, ?, ?>... modules) {
        super(
            TalonFX::new, TalonFX::new, CANcoder::new,
            Robot.bot.getDriveConfig().drivetrainConstants(),
            modules);

        gyroYaw = super.getPigeon2().getYaw();

        for (int i = 0; i < 4; i++) {
            TalonFX driveMotor = super.getModule(i).getDriveMotor();

            drivePositionSignals[i] = driveMotor.getPosition();
            driveVelocitySignals[i] = driveMotor.getVelocity();
        }
    }

    public DriveIOPhoenix() {
        this(
            Robot.bot.getDriveConfig().frontLeft(),
            Robot.bot.getDriveConfig().frontRight(),
            Robot.bot.getDriveConfig().backLeft(),
            Robot.bot.getDriveConfig().backRight());
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // Refresh all signals
        BaseStatusSignal.refreshAll(gyroYaw);
        BaseStatusSignal.refreshAll(drivePositionSignals);
        BaseStatusSignal.refreshAll(driveVelocitySignals);

        // Update drive inputs
        inputs.fromSwerveDriveState(super.getStateCopy());
        
        for (int i = 0; i < 4; i++) {
            inputs.drivePositionsRad[i] = Units.rotationsToRadians(drivePositionSignals[0].getValueAsDouble());
            inputs.driveVelocitiesRadPerSec[i] = Units.rotationsToRadians(driveVelocitySignals[0].getValueAsDouble());
        }
    }

    @Override
    public void setPose(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        super.resetRotation(angle);
    }

    @Override
    public void acceptVisionMeasurement(Pose2d poseEstimate, double timestamp, Matrix<N3, N1> stdDevs) {
        double newTimestamp = Utils.fpgaToCurrentTime(timestamp);

        if (stdDevs != null) {
            super.addVisionMeasurement(poseEstimate, newTimestamp, stdDevs);
        } else {
            super.addVisionMeasurement(poseEstimate, newTimestamp);
        }
    }

    @Override
    public void brake() {
        super.setControl(new SwerveDriveBrake());
    }

    @Override
    public void coast() {
        super.setControl(new SwerveDriveCoast());
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds) {
        super.setControl(DriveRequest.APPLY_ROBOT_SPEEDS.withSpeeds(speeds));
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {
        super.setControl(
            DriveRequest.APPLY_ROBOT_SPEEDS
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(moduleForcesX)
                .withWheelForceFeedforwardsY(moduleForcesY));
    }

    @Override
    public void runCharacterization(double output) {
        super.setControl(DriveRequest.RUN_CHARACTERIZATION.withVolts(output));
    }
}
