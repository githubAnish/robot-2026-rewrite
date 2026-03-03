package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.drive.DriveConstants;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;
import org.frogforce503.robot.subsystems.vision.Vision;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(JoystickUtil.class)
public class IntakeFuelFromGround extends Command {
    private final Drive drive;
    private final Vision vision;

    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;

    private final CommandXboxController xboxController;
    private final BooleanSupplier autoAssistEnabled;

    // Constants
    private final double kLookaheadTimeSec = 0.15;

    private final double kAssistMaxDistance = Units.inchesToMeters(90);
    private final double kMaxAssistStrafeSpeed = 0.7;
    private final double kMaxAssistOmega = 2.0;

    private final double kThetaAssistGain = 4.0;

    // Controllers
    private final PIDController lateralAssistController = new PIDController(1.2, 0.0, 0.0);

    // Sim
    private final IntakeSimulation intakeSimulation;

    public IntakeFuelFromGround(
        Drive drive,
        Vision vision,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        CommandXboxController xboxController,
        BooleanSupplier autoAssistEnabled,
        IntakeSimulation intakeSimulation
    ) {
        this.drive = drive;
        this.vision = vision;

        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;

        this.xboxController = xboxController;
        this.autoAssistEnabled = autoAssistEnabled;

        this.intakeSimulation = intakeSimulation;

        addRequirements(drive, intakePivot, intakeRoller);
    }

    @Override
    public void initialize() {
        intakePivot.setAngle(IntakePivotConstants.INTAKE);
        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);

        if (RobotBase.isSimulation()) {
            intakeSimulation.startIntake();
        }
    }

    @Override
    public void execute() {
        // Get inputs
        Pose2d robotPose = drive.getLookaheadPose(kLookaheadTimeSec);
        Translation2d target = vision.getFieldToBestCluster();

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

        // Assist logic (normal teleop drive if auto assist not wanted)
        double yError = 0.0;
        double yOutput = 0.0;
        double distance = 0.0;

        if (autoAssistEnabled.getAsBoolean()) {
            Translation2d toTargetField = target.minus(robotPose.getTranslation());
            Translation2d toTargetRobot = toTargetField.rotateBy(robotPose.getRotation().unaryMinus());

            distance = toTargetField.getNorm();
            double distanceScale = MathUtil.clamp((kAssistMaxDistance - distance) / kAssistMaxDistance, 0.0, 1.0);

            // Lateral assist
            yError = toTargetRobot.getY();
            double strafeOverride = MathUtil.clamp(1.0 - Math.abs(yVelocity), 0.0, 1.0);
            yOutput = lateralAssistController.calculate(yError, 0.0) * distanceScale * strafeOverride;
            yOutput = MathUtil.clamp(yOutput, -kMaxAssistStrafeSpeed, kMaxAssistStrafeSpeed);

            speeds.vyMetersPerSecond += yOutput;

            // Rotation assist
            double headingError = toTargetField.getAngle().minus(robotPose.getRotation()).getRadians();
            double omegaOverride = MathUtil.clamp(1.0 - Math.abs(omega), 0.0, 1.0);
            double omegaAssist = MathUtil.clamp(headingError * kThetaAssistGain, -kMaxAssistOmega, kMaxAssistOmega);

            speeds.omegaRadiansPerSecond += omegaAssist * distanceScale * omegaOverride;
        }

        // Apply speeds
        drive.runVelocity(speeds);

        // Log data
        Logger.recordOutput("IntakeFuelFromGround/YError", autoAssistEnabled.getAsBoolean() ? new double[] { yError } : new double[] {});
        Logger.recordOutput("IntakeFuelFromGround/YOutput", autoAssistEnabled.getAsBoolean() ? new double[] { yOutput } : new double[] {});
        Logger.recordOutput("IntakeFuelFromGround/Distance", autoAssistEnabled.getAsBoolean() ? new double[] { distance } : new double[] {});
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.stop();
        intakeRoller.stop();

        if (RobotBase.isSimulation()) {
            intakeSimulation.stopIntake();
        }
    }
}