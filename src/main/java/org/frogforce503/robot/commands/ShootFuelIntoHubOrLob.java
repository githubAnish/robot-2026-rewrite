package org.frogforce503.robot.commands;

import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.math.AllianceFlipUtil;
import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator;
import org.frogforce503.robot.subsystems.superstructure.ShotCalculator.ShotInfo;
import org.frogforce503.robot.subsystems.superstructure.feeder.Feeder;
import org.frogforce503.robot.subsystems.superstructure.feeder.FeederConstants;
import org.frogforce503.robot.subsystems.superstructure.flywheels.Flywheels;
import org.frogforce503.robot.subsystems.superstructure.flywheels.FlywheelsConstants;
import org.frogforce503.robot.subsystems.superstructure.hood.Hood;
import org.frogforce503.robot.subsystems.superstructure.hood.HoodConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(JoystickUtil.class)
public class ShootFuelIntoHubOrLob extends Command {
    private final Drive drive;
    private final Feeder feeder;
    private final Hood hood;
    private final Flywheels flywheels;
    private final GameViz gameViz;
    private final Supplier<Translation2d> linearVelocitySupplier;

    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            8.0,
            0.0,
            0.5,
            new Constraints(DriveConstants.maxOmega, DriveConstants.maxOmega * 0.7));

  private static final LoggedTunableNumber lockMetersPerSecondThreshold =
      new LoggedTunableNumber("DriveCommands/Launching/LockMetersPerSecThreshold", 0.1);
  private static final LoggedTunableNumber lockOmegaRadsPerSecThreshold =
      new LoggedTunableNumber("DriveCommands/Launching/LockOmegaRadsPerSecThreshold", 0.15);

  private static final LoggedTunableNumber driveLaunchMaxPolarVelocityRadPerSec =
      new LoggedTunableNumber("DriveCommands/Launching/MaxPolarVelocityRadPerSec", 0.6);
  private static final LoggedTunableNumber driveLauncherCORMinErrorDeg =
      new LoggedTunableNumber("DriveCommands/Launching/DriveLauncherCORMinErrorDeg", 15.0);
  private static final LoggedTunableNumber driveLauncherCORMaxErrorDeg =
      new LoggedTunableNumber("DriveCommands/Launching/DriveLauncherCORMaxErrorDeg", 30.0);

    public ShootFuelIntoHubOrLob(
        Drive drive,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        GameViz gameViz,
        CommandXboxController xboxController
    ) {
        this.drive = drive;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheels = flywheels;
        this.gameViz = gameViz;
        this.linearVelocitySupplier = () -> xboxController.getLinearVelocityFromJoysticks();

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive, feeder, hood, flywheels);
    }

    public ShootFuelIntoHubOrLob(
        Drive drive,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        GameViz gameViz
    ) {
        this.drive = drive;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheels = flywheels;
        this.gameViz = gameViz;
        this.linearVelocitySupplier = Translation2d::new;

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive, feeder, hood, flywheels);
    }

    @Override
    public void initialize() {
        thetaController.reset(
            drive.getAngle().getRadians(),
            drive.getFieldVelocity().omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        // Define shot parameters
        double hoodAngleRad = 0.0;
        double hoodVelocityRadPerSec = 0.0;
        double flywheelsVelocityRadPerSec = 0.0;

        // Get latest shot info
        ShotInfo shotInfo =
            ShotCalculator.getInstance().calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());

        switch (ShotCalculator.getInstance().getShotPreset()) {
            case NONE:
                hoodAngleRad = shotInfo.hoodAngleRad();
                hoodVelocityRadPerSec = shotInfo.hoodVelocityRadPerSec();
                flywheelsVelocityRadPerSec = shotInfo.flywheelsVelocityRadPerSec();
                break;

            case BATTER:
                hoodAngleRad = HoodConstants.BATTER;
                hoodVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.BATTER;
                break;

            case TRENCH:
                hoodAngleRad = HoodConstants.TRENCH;
                hoodVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.TRENCH;
                break;

            case DEPOT:
                hoodAngleRad = HoodConstants.DEPOT;
                hoodVelocityRadPerSec = 0.0;
                flywheelsVelocityRadPerSec = FlywheelsConstants.DEPOT;
                break;   
        }

        // Run PID controller
        double omegaOutput =
            thetaController.calculate(
                drive.getAngle().getRadians(),
                new State(shotInfo.driveAngle().getRadians(), shotInfo.driveVelocity())) + shotInfo.driveVelocity();

        // Calculate speeds
        Translation2d fieldRelativeLinearVelocity = linearVelocitySupplier.get().times(DriveConstants.maxLinearSpeed);

        if (AllianceFlipUtil.shouldFlip()) {
            fieldRelativeLinearVelocity = fieldRelativeLinearVelocity.times(-1.0);
        }

        // Limit if shooting, not lobbing
        if (FieldConstants.inAllianceZone(drive.getPose())) {
            // Calculate max linear velocity magnitude based on the max polar velocity
            double maxLinearVelocityMagnitude = Double.POSITIVE_INFINITY;
            double robotAngle =
                Math.abs(
                    AllianceFlipUtil.apply(FieldConstants.Hub.blueCenter.toTranslation2d())
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .minus(fieldRelativeLinearVelocity.getAngle())
                        .getRadians());

            double robotHubDistance = shotInfo.launcherToTargetDistanceNoLookahead();
            double hubAngle = driveLaunchMaxPolarVelocityRadPerSec.get() * ShotCalculator.getInstance().getNaiveTOF(robotHubDistance);
            double lookaheadAngle = Math.PI - robotAngle - hubAngle;

            // Calculate limit if triangle is valid (otherwise no limit)
            if (lookaheadAngle > 0) {
                double robotLookaheadDistance = robotHubDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle);
                maxLinearVelocityMagnitude = robotLookaheadDistance / ShotCalculator.getInstance().getNaiveTOF(robotHubDistance);
            }

            // Apply limit to velocity
            if (fieldRelativeLinearVelocity.getNorm() > maxLinearVelocityMagnitude) {
                fieldRelativeLinearVelocity =
                    fieldRelativeLinearVelocity.times(
                        maxLinearVelocityMagnitude / fieldRelativeLinearVelocity.getNorm());
            }
        }

        // Apply chassis speeds
        double corScalar =
            MathUtil.clamp(
                (Math.abs(shotInfo.driveAngle().minus(drive.getAngle()).getDegrees()) - driveLauncherCORMinErrorDeg.get()) / (driveLauncherCORMaxErrorDeg.get() - driveLauncherCORMinErrorDeg.get()),
                0.0,
                1.0);

        Translation2d launcherToRobot = HoodConstants.robotToHood.getTranslation().toTranslation2d().unaryMinus();

        ChassisSpeeds fieldRelativeSpeedsWithOffset =
            GeomUtil.transformVelocity(
                new ChassisSpeeds(
                    fieldRelativeLinearVelocity.getX(),
                    fieldRelativeLinearVelocity.getY(),
                    omegaOutput),
                launcherToRobot.times(1.0 - corScalar),
                drive.getAngle());

        // Apply brake
        boolean xLock =
            Math.hypot(fieldRelativeSpeedsWithOffset.vxMetersPerSecond, fieldRelativeSpeedsWithOffset.vyMetersPerSecond) < lockMetersPerSecondThreshold.get() &&
            Math.abs(fieldRelativeSpeedsWithOffset.omegaRadiansPerSecond) < lockOmegaRadsPerSecThreshold.get();
                    
        Logger.recordOutput("ShootFuelIntoHubOrLob/XLock", xLock);

        if (xLock) {
            drive.stopWithX();
        } else {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeSpeedsWithOffset, drive.getAngle()));
        }

        // Run subsystems
        hood.setAngle(hoodAngleRad, hoodVelocityRadPerSec);
        flywheels.setVelocity(flywheelsVelocityRadPerSec);

        // Check if shot feasible
        boolean isShotFeasible = ShotCalculator.getInstance().isShotFeasible();

        // Run feeder if shot feasible
        if (isShotFeasible) {
            feeder.setVelocity(FeederConstants.SHOOT);
        }

        // Simulate shooting
        if (RobotBase.isSimulation() && isShotFeasible) {
            gameViz.shootFuel(true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        hood.stop();
        flywheels.stop();
    }
}