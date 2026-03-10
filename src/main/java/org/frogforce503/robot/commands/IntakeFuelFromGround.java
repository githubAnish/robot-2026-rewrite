package org.frogforce503.robot.commands;

import java.util.function.BooleanSupplier;

import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.subsystems.drive.Drive;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRollerConstants;
import org.frogforce503.robot.subsystems.vision.Vision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(JoystickUtil.class)
public class IntakeFuelFromGround extends Command {
    private final Drive drive;
    private final Vision vision;

    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;

    private final GameViz gameViz;

    private final BooleanSupplier autoAssistEnabled;

    public IntakeFuelFromGround(
        Drive drive,
        Vision vision,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        GameViz gameViz,
        BooleanSupplier autoAssistEnabled
    ) {
        this.drive = drive;
        this.vision = vision;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.gameViz = gameViz;
        
        this.autoAssistEnabled = autoAssistEnabled;

        addRequirements(intakePivot, intakeRoller);
    }

    @Override
    public void initialize() {
        intakePivot.setAngle(IntakePivotConstants.INTAKE);
        intakeRoller.setVelocity(IntakeRollerConstants.INTAKE);

        if (RobotBase.isSimulation()) {
            gameViz.startIntake();
        }
    }

    @Override
    public void execute() {
        // Pose2d robotPose = drive.getPose();
        // Translation2d target = new Translation2d(); // Use object detection for vision.getFieldToBestCluster();

        // Assist logic (normal teleop drive if auto assist not wanted)
        if (autoAssistEnabled.getAsBoolean()) {

        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.setAngle(IntakePivotConstants.STOW);
        intakeRoller.stop();

        if (RobotBase.isSimulation()) {
            gameViz.stopIntake();
        }
    }
}