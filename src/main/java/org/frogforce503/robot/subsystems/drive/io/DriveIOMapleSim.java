package org.frogforce503.robot.subsystems.drive.io;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import org.frogforce503.lib.swerve.MapleSimSwerveDrivetrain;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.viz.SimConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;

@SuppressWarnings("unchecked")
public class DriveIOMapleSim extends DriveIOPhoenix {
    @Getter private final MapleSimSwerveDrivetrain drivetrain;
    private final Notifier simNotifier;

    private static final double kSimLoopPeriod = 0.002; // 2 ms

    public DriveIOMapleSim(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... modules) {
        super(
            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));

        drivetrain =
            new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                Kilograms.of(SimConstants.mass),
                Meters.of(DriveConstants.bumperLength),
                Meters.of(DriveConstants.bumperWidth),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                DriveConstants.wheelCOF,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                modules);

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