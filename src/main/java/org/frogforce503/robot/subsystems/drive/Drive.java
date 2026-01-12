package org.frogforce503.robot.subsystems.drive;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.vision.apriltag_detection.VisionMeasurement;
import org.frogforce503.robot.FieldInfo;
import org.frogforce503.robot.subsystems.drive.io.DriveIO;
import org.frogforce503.robot.subsystems.drive.io.DriveIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class Drive extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    // Viz
    @Getter private final DriveViz viz = new DriveViz();

    // State
    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    // Toggles
    @Getter private boolean robotRelative = false;
    @Getter private boolean slowMode = false;
    @Setter @Getter private double teleopTranslationScalar = 1.0;
    @Setter @Getter private double teleopRotationScalar = 1.0;
    @Setter @Getter private boolean coastAfterAutoEnd = false;

    public Drive(DriveIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        viz.update(inputs);

        Logger.recordOutput("Drive/TargetVelocity", targetSpeeds);
        Logger.recordOutput("Drive/Toggles/SlowModeEnabled", slowMode);
        Logger.recordOutput("Drive/Toggles/RobotRelative", robotRelative);

        // Record cycle time
        LoggedTracer.record("Drive");
    }

    // Toggles
    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public void toggleRobotRelative() {
        robotRelative = !robotRelative;
    }

    // Setters
    public void setPose(Pose2d pose) {
        io.setPose(pose);
    }

    public void setAngle(Rotation2d rotation) {
        io.setAngle(rotation);
    }

    public void resetRotation() {
        setAngle(
            FieldInfo.isRed()
                ? Rotation2d.kZero
                : Rotation2d.kPi);
    }
    
    // Adding vision measurements
    public void acceptVisionMeasurement(VisionMeasurement measurement) {
        io.acceptVisionMeasurement(
            measurement.pose(),
            measurement.timestamp(),
            measurement.standardDeviations());
    }

    // Getters
    public Pose2d getPose() {
        return inputs.Pose;
    }

    public Pose2d getFuturePose(double lookaheadTimeSec) {
        return getPose().exp(getRobotVelocity().toTwist2d(lookaheadTimeSec));
    }

    public ChassisSpeeds getRobotVelocity() {
        return inputs.Speeds;
    }

    public Rotation2d getAngle() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getAngle());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = inputs.drivePositionsRad[i];
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output +=
                Units.radiansToRotations(inputs.driveVelocitiesRadPerSec[i]) / 4.0;
        }
        return output;
    }

    public Rotation2d getGyroRotation() {
        return inputs.gyroAngle;
    }

    // Actions
    public void coast() {
        io.coast();
    }

    public void brake() {
        io.brake();
    }

    /** Runs a robot-relative ChassisSpeeds to the drivetrain. */
    public void runVelocity(ChassisSpeeds speeds) {
        io.runVelocity(speeds);
        this.targetSpeeds = speeds;
    }

    /** Runs a robot-relative ChassisSpeeds to the drivetrain with wheel force feedforwards in the X & Y direction. */
    public void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {
        io.runVelocity(speeds, moduleForcesX, moduleForcesY);
        this.targetSpeeds = speeds;
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        io.runCharacterization(output);
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }
}