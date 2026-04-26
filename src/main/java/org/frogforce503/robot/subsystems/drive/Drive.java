package org.frogforce503.robot.subsystems.drive;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.logging.LoggerUtil;
import org.frogforce503.lib.rebuilt.sim.TrenchCollisionSim;
import org.frogforce503.lib.swerve.MapleSimSwerveDrivetrain;
import org.frogforce503.robot.subsystems.drive.io.DriveIO;
import org.frogforce503.robot.subsystems.drive.io.DriveIOInputsAutoLogged;
import org.frogforce503.robot.subsystems.drive.io.DriveIOMapleSim;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class Drive extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    @Setter
    private TrenchCollisionSim trenchCollisionSim;

    @Getter
    private final DriveViz viz = new DriveViz();

    @Accessors(fluent = true)
    @Setter
    @Getter
    private boolean shouldCoastAfterAutoEnd = false;

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

    // Getters
    public Pose2d getPose() {
        return inputs.Pose;
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotVelocity() {
        return inputs.Speeds;
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getRotation());
    }

    public Rotation2d getGyroRotation() {
        return inputs.gyroAngle;
    }

    // Setters
    public void setPose(Pose2d pose) {
        io.setPose(pose);
    }

    public void setAngle(Rotation2d rotation) {
        io.setAngle(rotation);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        io.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    // Control Methods
    public void runVelocity(ChassisSpeeds speeds) {
        if (RobotBase.isSimulation()) {
            speeds = trenchCollisionSim.filterSpeeds(speeds, getPose(), getRotation());
        }
        io.runVelocity(speeds);
    }

    public void runVelocity(ChassisSpeeds speeds, double[] moduleForcesX, double[] moduleForcesY) {
        if (RobotBase.isSimulation()) {
            speeds = trenchCollisionSim.filterSpeeds(speeds, getPose(), getRotation());
        }
        io.runVelocity(speeds, moduleForcesX, moduleForcesY);
    }

    public void runCharacterization(double output) {
        io.runCharacterization(output);
    }

    // Stop Methods
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithX() {
        io.stopWithX();
        stop();
    }

    public void coast() {
        io.coast();
    }

    // Characterization
    public double[] getWheelRadiusCharacterizationPositionsRad() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = inputs.drivePositionsRad[i];
        }
        return values;
    }

    public double getFFCharacterizationVelocityRotPerSec() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += Units.radiansToRotations(inputs.driveVelocitiesRadPerSec[i]) / 4.0;
        }
        return output;
    }

    // Simulation
    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        if (io instanceof DriveIOMapleSim) {
            return ((DriveIOMapleSim) io).getDrivetrain();
        }
        return null;
    }
}