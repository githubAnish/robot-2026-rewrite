package org.frogforce503.robot.subsystems.drive;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.logging.LoggerUtil;
import org.frogforce503.lib.rebuilt.MapleSimUtil;
import org.frogforce503.lib.swerve.MapleSimSwerveDrivetrain;
import org.frogforce503.lib.vision.apriltagdetection.VisionMeasurement;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.io.DriveIO;
import org.frogforce503.robot.subsystems.drive.io.DriveIOInputsAutoLogged;
import org.frogforce503.robot.subsystems.drive.io.DriveIOMapleSim;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
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

    // Setters
    public void setPose(Pose2d pose) {
        io.setPose(pose);
    }

    public void setAngle(Rotation2d rotation) {
        io.setAngle(rotation);
    }

    public void resetRotation() {
        setAngle(
            FieldConstants.isRed()
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

    public Rotation2d getAngle() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotVelocity() {
        return inputs.Speeds;
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
            output += Units.radiansToRotations(inputs.driveVelocitiesRadPerSec[i]) / 4.0;
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

    /** Stops the drivetrain by aligning the modules in X manner. */
    public void brake() {
        io.brake();
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /** Runs a robot-relative ChassisSpeeds to the drivetrain. */
    public void runVelocity(ChassisSpeeds speeds) {
        if (RobotBase.isSimulation()) {
            speeds = MapleSimUtil.limitVelocityOverBumps(getPose().getTranslation(), speeds);
        }

        io.runVelocity(speeds);
    }

    /** Runs a robot-relative ChassisSpeeds to the drivetrain with wheel force feedforwards in the X & Y direction. */
    public void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {
        if (RobotBase.isSimulation()) {
            speeds = MapleSimUtil.limitVelocityOverBumps(getPose().getTranslation(), speeds);
        }

        io.runVelocity(speeds, moduleForcesX, moduleForcesY);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        io.runCharacterization(output);
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        if (io instanceof DriveIOMapleSim) {
            return ((DriveIOMapleSim) io).getDrivetrain();
        }
        return null;
    }
}