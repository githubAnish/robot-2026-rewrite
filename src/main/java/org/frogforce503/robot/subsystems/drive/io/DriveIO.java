package org.frogforce503.robot.subsystems.drive.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface DriveIO {
    @AutoLog
    class DriveIOInputs extends SwerveDriveState {
        public double[] drivePositionsRad = new double[4];
        public double[] driveVelocitiesRadPerSec = new double[4];
        public Rotation2d gyroAngle = Rotation2d.kZero;

        public void fromSwerveDriveState(SwerveDriveState other) {
            this.Pose = other.Pose;
            this.SuccessfulDaqs = other.SuccessfulDaqs;
            this.FailedDaqs = other.FailedDaqs;
            this.ModuleStates = other.ModuleStates;
            this.ModuleTargets = other.ModuleTargets;
            this.Speeds = other.Speeds;
            this.OdometryPeriod = other.OdometryPeriod;
        }
    }

    default void updateInputs(DriveIOInputs inputs) {}

    default void setPose(Pose2d pose) {}

    default void setAngle(Rotation2d angle) {}

    default void acceptVisionMeasurement(Pose2d poseEstimate, double timestamp, Matrix<N3, N1> stdDevs) {}

    default void brake() {}

    default void coast() {}

    default void runVelocity(ChassisSpeeds speeds) {}

    default void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {}

    default void runCharacterization(double output) {}
}
