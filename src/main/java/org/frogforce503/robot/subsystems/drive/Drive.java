package org.frogforce503.robot.subsystems.drive;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.logging.LoggerUtil;
import org.frogforce503.lib.swerve.MapleSimSwerveDrivetrain;
import org.frogforce503.lib.vision.apriltagdetection.VisionMeasurement;
import org.frogforce503.robot.subsystems.drive.io.DriveIO;
import org.frogforce503.robot.subsystems.drive.io.DriveIOInputsAutoLogged;
import org.frogforce503.robot.subsystems.drive.io.DriveIOMapleSim;
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

    @Getter private final DriveViz viz = new DriveViz();

    @Setter @Getter private boolean coastAfterAutoEnd = false;

    public Drive(DriveIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggerUtil.recordCurrentCommand(this);

        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        viz.update(inputs);

        // Record cycle time
        LoggedTracer.record("Drive");
    }

    public Pose2d getPose() {
        return inputs.Pose;
    }

    public Rotation2d getAngle() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotVelocity() {
        return inputs.Speeds;
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getAngle());
    }

    public Rotation2d getGyroRotation() {
        return inputs.gyroAngle;
    }

    public void setPose(Pose2d pose) {
        io.setPose(pose);
    }

    public void setAngle(Rotation2d rotation) {
        io.setAngle(rotation);
    }

    /** Runs a robot-relative ChassisSpeeds to the drivetrain. */
    public void runVelocity(ChassisSpeeds speeds) {
        io.runVelocity(speeds);
    }

    /** Runs a robot-relative ChassisSpeeds to the drivetrain with wheel force feedforwards in the X & Y direction. */
    public void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {
        io.runVelocity(speeds, moduleForcesX, moduleForcesY);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        io.runCharacterization(output);
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        io.stopWithX();
        stop();
    }

    /** Stops the drive and turns the modules to an O arrangement to resist movement. */
    public void stopWithO() {
        io.stopWithO();
        stop();
    }

    public void coast() {
        io.coast();
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
            output += Units.radiansToRotations(inputs.driveVelocitiesRadPerSec[i]) / 4.0;
        }
        return output;
    }

    public void acceptVisionMeasurement(VisionMeasurement measurement) {
        io.acceptVisionMeasurement(
            measurement.pose(),
            measurement.timestamp(),
            measurement.standardDeviations());
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        if (io instanceof DriveIOMapleSim) {
            return ((DriveIOMapleSim) io).getDrivetrain();
        }
        return null;
    }
}