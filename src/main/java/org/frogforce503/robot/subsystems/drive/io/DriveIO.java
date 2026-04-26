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
            this.Speeds = other.Speeds;
            this.ModuleStates = other.ModuleStates;
            this.ModuleTargets = other.ModuleTargets;
            this.ModulePositions = other.ModulePositions;
            this.RawHeading = other.RawHeading;
            this.Timestamp = other.Timestamp;
            this.OdometryPeriod = other.OdometryPeriod;
            this.SuccessfulDaqs = other.SuccessfulDaqs;
            this.FailedDaqs = other.FailedDaqs;
        }
    }

    default void updateInputs(DriveIOInputs inputs) {}

    default void setPose(Pose2d pose) {}

    default void setAngle(Rotation2d angle) {}

    default void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    default void stopWithX() {}

    default void coast() {}

    default void runVelocity(ChassisSpeeds speeds) {}

    default void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {}

    default void runCharacterization(double output) {}
}
