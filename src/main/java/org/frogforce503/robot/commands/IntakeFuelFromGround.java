package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.Superstructure;
import org.frogforce503.robot.subsystems.superstructure.indexer.Indexer;
import org.frogforce503.robot.subsystems.superstructure.indexer.IndexerConstants;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(JoystickUtil.class)
public class IntakeFuelFromGround extends Command {
    private final Drive drive;
    private final Vision vision;

    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;

    private final CommandXboxController xboxController;
    private final BooleanSupplier autoAssistEnabled;

    // Constants
    private final double kLookaheadTimeSec = 0.15;

    private final double kAssistMaxDistance = Units.inchesToMeters(90);
    private final double kMaxAssistStrafe = 0.7;     // m/s
    private final double kMaxAssistOmega  = 2.0;     // rad/s

    private final double kThetaAssistGain = 4.0;

    // Controllers
    private final PIDController lateralAssistController = new PIDController(1.2, 0.0, 0.0);

    public IntakeFuelFromGround(
        Drive drive,
        Vision vision,
        Superstructure superstructure,
        CommandXboxController xboxController,
        BooleanSupplier autoAssistEnabled
    ) {
        this.drive = drive;
        this.vision = vision;

        this.intakePivot = superstructure.getIntakePivot();
        this.intakeRoller = superstructure.getIntakeRoller();
        this.indexer = superstructure.getIndexer();

        this.xboxController = xboxController;
        this.autoAssistEnabled = autoAssistEnabled;

        addRequirements(drive, intakePivot, intakeRoller, indexer);
    }

    @Override
    public void initialize() {
        if (indexer.isCompressed()) {
            return;
        }

        intakePivot.setAngle(IntakePivotConstants.INTAKE);
        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);
        indexer.setVelocity(IndexerConstants.INTAKE);
    }

    @Override
    public void execute() {
        // Get inputs
        Pose2d robotPose = drive.getLookaheadPose(kLookaheadTimeSec);
        boolean visionHasValidFuelTarget = true; // TODO assume true for now, change in vision

        // Calculate default teleop velocities
        Translation2d driverLinearVelocity = xboxController.getLinearVelocityFromJoysticks();
        double driverOmega = xboxController.getOmegaFromJoysticks();

        double xVelocity = driverLinearVelocity.getX() * DriveConstants.maxLinearSpeed;
        double yVelocity = driverLinearVelocity.getY() * DriveConstants.maxLinearSpeed;
        double omega = driverOmega * DriveConstants.maxOmega;

        ChassisSpeeds speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                omega,
                drive.getAngle());

        // Normal teleop drive if auto assist not wanted or vision doesn't have good view of fuel
        if (!autoAssistEnabled.getAsBoolean() || !visionHasValidFuelTarget) {
            drive.runVelocity(speeds);
            return;
        }

        // Assist logic
        Pose2d targetPose = Pose2d.kZero; // TODO assume a fixed point for now, change in vision

        Translation2d toTargetField = targetPose.getTranslation().minus(robotPose.getTranslation());
        Translation2d toTargetRobot = toTargetField.rotateBy(robotPose.getRotation().unaryMinus());

        double distance = toTargetField.getNorm();
        double distanceScale = MathUtil.clamp((kAssistMaxDistance - distance) / kAssistMaxDistance, 0.0, 1.0);

        // Lateral assist
        double yError = toTargetRobot.getY();
        double strafeOverride = MathUtil.clamp(1.0 - Math.abs(yVelocity), 0.0, 1.0);
        double yOutput = lateralAssistController.calculate(yError, 0.0) * distanceScale * strafeOverride;
        yOutput = MathUtil.clamp(yOutput, -kMaxAssistStrafe, kMaxAssistStrafe);

        speeds.vyMetersPerSecond += yOutput;

        // Rotation assist
        double headingError = toTargetField.getAngle().minus(robotPose.getRotation()).getRadians();
        double omegaOverride = MathUtil.clamp(1.0 - Math.abs(omega), 0.0, 1.0);
        double omegaAssist = MathUtil.clamp(headingError * kThetaAssistGain, -kMaxAssistOmega, kMaxAssistOmega);

        speeds.omegaRadiansPerSecond += omegaAssist * distanceScale * omegaOverride;

        // Apply speeds
        drive.runVelocity(speeds);

        // Log data
        Logger.recordOutput("IntakeFuelFromGround/YError", yError);
        Logger.recordOutput("IntakeFuelFromGround/YOutput", yOutput);
        Logger.recordOutput("IntakeFuelFromGround/Distance", distance);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.stop();
        intakeRoller.stop();
        indexer.setVelocity(IndexerConstants.SLOW_MIX); // TODO maybe slow mixing in order to prep for shooting
    }
}
