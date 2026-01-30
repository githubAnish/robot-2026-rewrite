package org.frogforce503.robot.subsystems.drive.io;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import org.frogforce503.lib.swerve.MapleSimSwerveDrivetrain;
import org.frogforce503.robot.subsystems.drive.DriveConstants;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

@SuppressWarnings("unchecked")
public class DriveIOMapleSim extends DriveIOPhoenix {
    // Requirements
    private final MapleSimSwerveDrivetrain drivetrain;
    private Notifier simNotifier;

    // Constants
    private static final double kSimLoopPeriod = 0.002; // 2 ms

    public DriveIOMapleSim(SwerveModuleConstants<?, ?, ?>... modules) {
        super(
            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));

        drivetrain =
            new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                Pounds.of(132.2), // from robot-2025 Choreo settings
                Inches.of(33),
                Inches.of(33),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                1.2,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                DriveConstants.frontLeft,
                DriveConstants.frontRight,
                DriveConstants.backLeft,
                DriveConstants.backRight);

        simNotifier = new Notifier(drivetrain::update);
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public DriveIOMapleSim() {
        this(
            DriveConstants.frontLeft,
            DriveConstants.frontRight,
            DriveConstants.backLeft,
            DriveConstants.backRight);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        super.updateInputs(inputs);
        inputs.Pose = drivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        drivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        Timer.delay(0.05); // wait for simulation to update
        super.setPose(pose);
    }
}