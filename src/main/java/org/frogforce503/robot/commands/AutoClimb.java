package org.frogforce503.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickUtil;
import org.frogforce503.robot.GameViz;
import org.frogforce503.robot.commands.drive.DriveToPose;
import org.frogforce503.robot.constants.field.FieldConstants;
import org.frogforce503.robot.subsystems.climber.Climber;
import org.frogforce503.robot.subsystems.climber.ClimberConstants;
import org.frogforce503.robot.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;

// maybe use the linear velocity supplier and omega supplier by drivetopose and feed the path following output
// maybe its not needed
@ExtensionMethod(JoystickUtil.class)
public class AutoClimb extends SequentialCommandGroup {
    private final Drive drive;
    private final Climber climber;
    private final GameViz gameViz;

    private Supplier<Translation2d> linearVelocitySupplier = Translation2d::new;
    private DoubleSupplier omegaSupplier = () -> 0.0;

    @Setter private boolean disableAutoDrive = false;

    public AutoClimb(Drive drive, Climber climber, GameViz gameViz) {
        this.drive = drive;
        this.climber = climber;
        this.gameViz = gameViz;

        command();
    }
    
    public AutoClimb(
        Drive drive,
        Climber climber,
        GameViz gameViz,
        CommandXboxController xboxController
    ) {
        this.drive = drive;
        this.climber = climber;
        this.gameViz = gameViz;
        
        this.linearVelocitySupplier = () -> xboxController.getLinearVelocityFromJoysticks();
        this.omegaSupplier = () -> xboxController.getOmegaFromJoysticks();

        command();
    }

    private void command() {
        addCommands(
            Commands.runOnce(() -> climber.setHeight(ClimberConstants.maxHeight)),
            Commands.parallel(
                drive(true),
                Commands.waitUntil(() -> climber.isAtHeight(ClimberConstants.maxHeight, ClimberConstants.tolerance))
            ),
            drive(false),
            simStartClimb(),
            Commands.runOnce(() -> climber.setHeight(ClimberConstants.minHeight)),
            simClimb()
        );
    }

    private Command drive(boolean isPreClimb) {
        final Supplier<Pose2d> target =
            () ->
                isPreClimb
                    ? FieldConstants.Tower.getPreClimbPose(drive.getPose())
                    : FieldConstants.Tower.getClimbPose(drive.getPose());

        final DriveToPose driveToPose =
            RobotState.isTeleop()
                ? new DriveToPose(drive, target, linearVelocitySupplier, omegaSupplier)
                : new DriveToPose(drive, target);

        return driveToPose.unless(() -> disableAutoDrive);
    }

    private Command simStartClimb() {
        return Commands.runOnce(gameViz::startClimb).onlyIf(RobotBase::isSimulation);
    }

    private Command simClimb() {
        return
            Commands.sequence(
                Commands.run(gameViz::climb),
                Commands.waitUntil(() -> climber.isAtHeight(ClimberConstants.minHeight, ClimberConstants.tolerance)),
                Commands.runOnce(gameViz::stopClimb)
            )
            .onlyIf(RobotBase::isSimulation);
    }
}